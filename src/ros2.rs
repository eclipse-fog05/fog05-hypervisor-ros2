/*********************************************************************************
* Copyright (c) 2018,2021 ADLINK Technology Inc.
*
* This program and the accompanying materials are made available under the
* terms of the Eclipse Public License 2.0 which is available at
* http://www.eclipse.org/legal/epl-2.0, or the Apache Software License 2.0
* which is available at https://www.apache.org/licenses/LICENSE-2.0.
*
* SPDX-License-Identifier: EPL-2.0 OR Apache-2.0
* Contributors:
*   ADLINK fog05 team, <fog05@adlink-labs.tech>
*********************************************************************************/
#![allow(unused)]
#![allow(unused_variables)]
#![allow(unused_mut)]

use std::collections::HashMap;
use std::fs::File;
use std::process::{Command, Stdio};
use std::time::Duration;

use async_std::prelude::*;
use async_std::sync::{Arc, Mutex};
use async_std::task;

use nix::sys::signal::{kill, Signal};
use nix::unistd::Pid;

use psutil::process::{processes, Process, ProcessError, Status as ProcessStatus};

use znrpc_macros::znserver;
use zrpc::ZNServe;

use fog05_sdk::agent::{AgentPluginInterfaceClient, OSClient};
use fog05_sdk::fresult::{FError, FResult};
use fog05_sdk::im::fdu::*;
use fog05_sdk::im::fdu::{FDUDescriptor, FDURecord, FDUState};
use fog05_sdk::plugins::{HypervisorPlugin, NetworkingPluginClient};
use fog05_sdk::types::PluginKind;

use uuid::Uuid;

use crate::types::{
    deserialize_ros2_specific_descriptor, deserialize_ros2_specific_info,
    serialize_ros2_specific_info, ROS2AppKind, ROS2HVSpecificInfo, ROS2Hypervisor,
};

#[znserver]
impl HypervisorPlugin for ROS2Hypervisor {
    async fn define_fdu(&mut self, fdu: FDUDescriptor) -> FResult<FDURecord> {
        log::debug!("Define FDU {:?}", fdu);

        if fdu.hypervisor_specific.is_none() {
            return Err(FError::MalformedDescriptor);
        }

        log::trace!("Get node UUID");
        let node_uuid = self.agent.as_ref().unwrap().get_node_uuid().await??;
        log::trace!("Get RwLock");
        let mut guard = self.fdus.write().await;
        log::trace!("Creating instance UUID");
        let instance_uuid = Uuid::new_v4();
        log::trace!("Creating instance object");
        let instance = FDURecord {
            uuid: instance_uuid,
            fdu_uuid: fdu.uuid.unwrap(), //this is present because it is populated by the agent/orchestrator
            node: node_uuid,
            interfaces: Vec::new(),
            connection_points: Vec::new(),
            status: FDUState::DEFINED,
            error: None,
            hypervisor_specific: None,
            restarts: 0,
        };
        log::trace!("Add instance to local state");
        guard.fdus.insert(instance_uuid, instance.clone());
        log::trace!("Add instance in zenoh");
        self.connector.local.add_instance(&instance).await?;
        log::debug!("Instance status {:?}", instance.status);
        Ok(instance)
    }

    async fn undefine_fdu(&mut self, instance_uuid: Uuid) -> FResult<Uuid> {
        log::debug!("Undefine FDU {:?}", instance_uuid);
        let node_uuid = self.agent.as_ref().unwrap().get_node_uuid().await??;
        let instance = self.connector.local.get_instance(instance_uuid).await?;
        match instance.status {
            FDUState::DEFINED | FDUState::ERROR(_) => {
                let mut guard = self.fdus.write().await;
                guard.fdus.remove(&instance_uuid);
                self.connector.local.remove_instance(instance_uuid).await?;
                log::debug!("Instance status {} undefined", instance_uuid);
                Ok(instance_uuid)
            }
            _ => Err(FError::TransitionNotAllowed),
        }
    }

    async fn configure_fdu(&mut self, instance_uuid: Uuid) -> FResult<Uuid> {
        log::debug!("Configure FDU {:?}", instance_uuid);
        log::trace!("Get node UUID");
        let node_uuid = self.agent.as_ref().unwrap().get_node_uuid().await??;
        log::trace!("Get instance");
        let mut instance = self.connector.local.get_instance(instance_uuid).await?;
        log::trace!("Check FDU status: {:?}", instance.status);
        match instance.status {
            FDUState::DEFINED | FDUState::ERROR(_) => {
                let mut guard = self.fdus.write().await; //taking lock

                // Here we should create the network interfaces
                // connection points, etc...
                // dummy ask the creation of a namespace for FDU networking
                let descriptor = self
                    .agent
                    .as_ref()
                    .unwrap()
                    .fdu_info(instance.fdu_uuid)
                    .await??;

                let hv_info = deserialize_ros2_specific_descriptor(
                    &descriptor
                        .hypervisor_specific
                        .unwrap()
                        .into_bytes()
                        .as_slice(),
                )?;

                // Creating network namespace only if isolation feature is enabled
                let fdu_ns: Option<Uuid> = if cfg!(feature = "isolation") {
                    let ns = self
                        .net
                        .as_ref()
                        .unwrap()
                        .create_network_namespace()
                        .await??;
                    Some(ns.uuid)
                } else {
                    None
                };

                let instance_path = self
                    .get_run_path()
                    .join(format!("{}", instance_uuid))
                    .to_str()
                    .ok_or(FError::EncodingError)?
                    .to_string();

                let mut hv_specific = ROS2HVSpecificInfo {
                    pid: 0,
                    env: hv_info.env.clone(),
                    instance_path,
                    image_folder: None,
                    instance_files: Vec::new(),
                    netns: fdu_ns,
                };

                //creating instance path
                self.os
                    .as_ref()
                    .unwrap()
                    .create_dir(hv_specific.clone().instance_path)
                    .await??;

                // Getting the image
                // ROS2 Plugin expects images to be packaged as .tar.gz
                // once the download is complete the image will be
                // decompressed
                if let Some(img) = descriptor.image {
                    let uri = url::Url::parse(&img.uri.clone())
                        .map_err(|e| FError::HypervisorError(format!("{}", e)))?;
                    let img_uri = img.uri.clone();
                    let splitter_uri = img_uri.split('/').collect::<Vec<&str>>();
                    let f_name = splitter_uri.last().ok_or(FError::NotFound)?;
                    let f_path = self
                        .get_run_path()
                        .join(format!("{}", instance_uuid))
                        .join(f_name.to_string())
                        .to_str()
                        .ok_or(FError::EncodingError)?
                        .to_string();
                    log::trace!("Image {} will be downloaded in {}", uri, f_path);
                    self.os
                        .as_ref()
                        .unwrap()
                        .download_file(uri, f_path.clone())
                        .await??;

                    let img_name = match f_name.strip_suffix(".tar.gz") {
                        Some(x) => x,
                        None => {
                            return Err(FError::HypervisorError(
                                "Image is not packaged as .tar.gz".to_string(),
                            ));
                        }
                    };

                    let img_folder_path = self
                        .get_run_path()
                        .join(format!("{}", instance_uuid))
                        .join(img_name.to_string())
                        .to_str()
                        .ok_or(FError::EncodingError)?
                        .to_string();

                    log::trace!("Expected image folder {}", img_folder_path);

                    let cmd = format!(
                        "tar -xzf {} -C {}",
                        f_path,
                        hv_specific.clone().instance_path
                    );

                    log::trace!("Decompressing command {}", cmd);

                    self.os.as_ref().unwrap().execute_command(cmd).await??;

                    hv_specific.image_folder = Some(img_folder_path);
                }

                // Adding hv specific info
                instance.hypervisor_specific = Some(serialize_ros2_specific_info(&hv_specific)?);
                //

                log::trace!("Created instance info {:?}", hv_specific);

                let mut interfaces: Vec<FDURecordInterface> = Vec::new();

                let mut cps: HashMap<String, FDURecordConnectionPoint> = HashMap::new();

                //Creating just an interface to be attached to the default vnet

                if cfg!(feature = "isolation") {
                    let viface_config = fog05_sdk::types::VirtualInterfaceConfig {
                        if_name: "eth0".to_string(),
                        kind: fog05_sdk::types::VirtualInterfaceConfigKind::VETH,
                    };
                    let viface = self
                        .net
                        .as_ref()
                        .unwrap()
                        .create_virtual_interface_in_namespace(viface_config, fdu_ns.unwrap())
                        .await??;

                    log::trace!("Created virtual interface {:?}", viface);

                    let pair = match viface.kind {
                        fog05_sdk::types::VirtualInterfaceKind::VETH(info) => info.pair,
                        _ => return Err(FError::MalformedDescriptor),
                    };

                    // Moving pair interface into default network namespace
                    self.net
                        .as_ref()
                        .unwrap()
                        .move_interface_into_default_namespace(pair)
                        .await??;

                    // Attaching interface to default network virtual bridge
                    self.net
                        .as_ref()
                        .unwrap()
                        .attach_interface_to_bridge(pair, Uuid::nil())
                        .await??;

                    // Getting address with DHCP
                    let net_client = self.net.as_ref().unwrap().clone();
                    let face_uuid = viface.uuid;
                    task::spawn(async move {
                        let r = net_client
                            .assing_address_to_interface(face_uuid, None)
                            .await;
                        log::trace!("Assign address result: {:?}", r);
                    });

                    let fdu_intf = FDURecordInterface {
                        name: "eth0".to_string(),
                        kind: fog05_sdk::im::fdu::InterfaceKind::VIRTUAL,
                        mac_address: None,
                        cp_uuid: None,
                        intf_uuid: viface.uuid,
                        virtual_interface: FDURecordVirtualInterface {
                            vif_kind: VirtualInterfaceKind::E1000,
                            bandwidht: None,
                        },
                    };
                    interfaces.push(fdu_intf)
                }
                // Not creating any connection point or interface
                // for cp in descriptor.connection_points {
                //     let vcp = self
                //         .net
                //         .as_ref()
                //         .unwrap()
                //         .create_connection_point()
                //         .await??;

                //     let fdu_cp = FDURecordConnectionPoint {
                //         uuid: vcp.uuid,
                //         id: cp.id.clone(),
                //     };

                //     log::trace!("Created connection point {:?} {:?}", vcp, fdu_cp);

                //     cps.insert(cp.id, fdu_cp);
                //     // we should ask the connection of the cp to the virtual network here
                // }

                // for intf in descriptor.interfaces {
                //     let viface_config = fog05_sdk::types::VirtualInterfaceConfig {
                //         if_name: intf.name.clone(),
                //         kind: fog05_sdk::types::VirtualInterfaceConfigKind::VETH,
                //     };
                //     let viface = self
                //         .net
                //         .as_ref()
                //         .unwrap()
                //         .create_virtual_interface_in_namespace(viface_config, fdu_ns.uuid)
                //         .await??;

                //     log::trace!("Created virtual interface {:?}", viface);

                //     let pair = match viface.kind {
                //         fog05_sdk::types::VirtualInterfaceKind::VETH(info) => info.pair,
                //         _ => return Err(FError::MalformedDescriptor),
                //     };
                //     let cp_uuid = match intf.cp_id {
                //         Some(cp_id) => {
                //             let cp = cps.get(&cp_id).ok_or(FError::NotFound)?;
                //             self.net
                //                 .as_ref()
                //                 .unwrap()
                //                 .bind_interface_to_connection_point(pair, cp.uuid)
                //                 .await??;
                //             Some(cp.uuid)
                //         }
                //         None => {
                //             self.net
                //                 .as_ref()
                //                 .unwrap()
                //                 .move_interface_into_default_namespace(pair)
                //                 .await??;
                //             None
                //         }
                //     };

                //     // dummy hv creates all the faces as veth pairs
                //     let fdu_intf = FDURecordInterface {
                //         name: intf.name,
                //         kind: intf.kind,
                //         mac_address: intf.mac_address,
                //         cp_uuid,
                //         intf_uuid: viface.uuid,
                //         virtual_interface: FDURecordVirtualInterface {
                //             vif_kind: VirtualInterfaceKind::E1000,
                //             bandwidht: None,
                //         },
                //     };
                //     interfaces.push(fdu_intf)
                // }

                instance.interfaces = interfaces;
                instance.connection_points = cps.into_iter().map(|(_, v)| v).collect();
                instance.status = FDUState::CONFIGURED;
                self.connector.local.add_instance(&instance).await?;
                log::debug!("Instance status {:?}", instance.status);
                guard.fdus.insert(instance_uuid, instance);
                Ok(instance_uuid)
            }
            _ => Err(FError::TransitionNotAllowed),
        }
    }

    async fn clean_fdu(&mut self, instance_uuid: Uuid) -> FResult<Uuid> {
        log::debug!("Clean FDU {:?}", instance_uuid);
        let node_uuid = self.agent.as_ref().unwrap().get_node_uuid().await??;
        let mut instance = self.connector.local.get_instance(instance_uuid).await?;
        match instance.status {
            FDUState::CONFIGURED | FDUState::ERROR(_) => {
                let mut guard = self.fdus.write().await;

                let mut hv_specific = deserialize_ros2_specific_info(
                    &instance.clone().hypervisor_specific.unwrap().as_slice(),
                )?;

                let descriptor = self
                    .agent
                    .as_ref()
                    .unwrap()
                    .fdu_info(instance.fdu_uuid)
                    .await??;

                for iface in instance.interfaces {
                    self.net
                        .as_ref()
                        .unwrap()
                        .delete_virtual_interface(iface.intf_uuid)
                        .await??;
                    log::trace!("Deleted virtual interface {:?}", iface);
                }

                self.net
                    .as_ref()
                    .unwrap()
                    .delete_network_namespace(hv_specific.netns.unwrap())
                    .await??;

                // for cp in instance.connection_points {
                //     self.net
                //         .as_ref()
                //         .unwrap()
                //         .delete_connection_point(cp.uuid)
                //         .await??;
                //     log::trace!("Deletted connection point {:?}", cp);
                // }

                // removing all instance files
                for filename in hv_specific.clone().instance_files {
                    match std::fs::remove_file(filename.clone()) {
                        Ok(_) => log::trace!("Removed {}", filename),
                        Err(e) => log::warn!("file {} {}", filename, e),
                    }
                }

                if let Some(img) = descriptor.image {
                    let img_uri = img.uri;
                    let splitter_uri = img_uri.split('/').collect::<Vec<&str>>();
                    let f_name = splitter_uri.last().ok_or(FError::NotFound)?;
                    let f_path = self
                        .get_run_path()
                        .join(format!("{}", instance_uuid))
                        .join(f_name.to_string())
                        .to_str()
                        .ok_or(FError::EncodingError)?
                        .to_string();
                    match std::fs::remove_file(f_path.clone()) {
                        Ok(_) => log::trace!("Removed {}", f_path),
                        Err(e) => log::warn!("file {} {}", f_path, e),
                    }

                    let img_name = match f_name.strip_suffix(".tar.gz") {
                        Some(x) => x,
                        None => {
                            return Err(FError::HypervisorError(
                                "Image is not packaged as .tar.gz".to_string(),
                            ));
                        }
                    };

                    let img_folder_path = self
                        .get_run_path()
                        .join(format!("{}", instance_uuid))
                        .join(img_name.to_string())
                        .to_str()
                        .ok_or(FError::EncodingError)?
                        .to_string();
                    match std::fs::remove_dir_all(img_folder_path.clone()) {
                        Ok(_) => log::trace!("Removed {}", img_folder_path),
                        Err(e) => log::warn!("file {} {}", img_folder_path, e),
                    }
                };

                //removing instance directory
                self.os
                    .as_ref()
                    .unwrap()
                    .rm_dir(hv_specific.clone().instance_path)
                    .await??;

                hv_specific.instance_files = Vec::new();
                hv_specific.env = HashMap::new();

                instance.hypervisor_specific = Some(serialize_ros2_specific_info(&hv_specific)?);
                instance.interfaces = Vec::new();
                instance.connection_points = Vec::new();
                instance.status = FDUState::DEFINED;
                self.connector.local.add_instance(&instance).await?;
                log::debug!("Instance status {:?}", instance.status);
                guard.fdus.insert(instance_uuid, instance);

                Ok(instance_uuid)
            }
            _ => Err(FError::TransitionNotAllowed),
        }
    }

    async fn start_fdu(&mut self, instance_uuid: Uuid) -> FResult<Uuid> {
        log::debug!("Start FDU {:?}", instance_uuid);
        let node_uuid = self.agent.as_ref().unwrap().get_node_uuid().await??;
        let mut instance = self.connector.local.get_instance(instance_uuid).await?;
        log::trace!("Instance status {:?}", instance.status);
        match instance.status {
            FDUState::CONFIGURED | FDUState::ERROR(_) => {
                let mut guard = self.fdus.write().await;

                let descriptor = self
                    .agent
                    .as_ref()
                    .unwrap()
                    .fdu_info(instance.fdu_uuid)
                    .await??;

                let hv_info = deserialize_ros2_specific_descriptor(
                    &descriptor.hypervisor_specific.unwrap().into_bytes(),
                )?;

                let mut hv_specific = deserialize_ros2_specific_info(
                    &instance.clone().hypervisor_specific.unwrap().as_slice(),
                )?;

                let mut cmd = if cfg!(feature = "isolation") {
                    let ns = self
                        .net
                        .as_ref()
                        .unwrap()
                        .get_network_namespace(hv_specific.netns.unwrap())
                        .await??;

                    let mut cmd = Command::new("fos-prepare-ros2-isolate");
                    cmd.arg("--netns");
                    cmd.arg(ns.ns_name);
                    cmd.arg("--distro");
                    cmd.arg(self.config.ros2_distro.clone());
                    cmd.arg("--rmw");
                    cmd.arg(self.config.rmw.clone());
                    cmd.arg("--app-path");
                    cmd.arg(hv_specific.image_folder.clone().ok_or(FError::NotFound)?);
                    cmd.arg("--app-name");
                    cmd.arg(hv_info.app_name);
                    cmd.arg("--cmd");
                    cmd.arg(hv_info.entry_point);
                    match hv_info.kind {
                        ROS2AppKind::LAUNCH => {
                            cmd.arg("--launch");
                        }
                        ROS2AppKind::RUN => (),
                    };

                    let args = hv_info.args.join(" ");
                    cmd.arg("--args");
                    cmd.arg(args);

                    cmd
                } else {
                    return Err(FError::Unimplemented);
                    // let mut cmd = Command::new(hv_info.cmd);
                    // for arg in hv_info.args {
                    //     cmd.arg(arg);
                    // }
                    // cmd
                };

                cmd.envs(hv_specific.env.clone());

                // rearranging stdin,stdout, stderr
                cmd.stdin(Stdio::null());

                // creating file for stdout and stderr
                let out_filename = format!("{}/{}.out", hv_specific.instance_path, instance_uuid);
                let err_filename = format!("{}/{}.out", hv_specific.instance_path, instance_uuid);

                hv_specific.instance_files.push(out_filename.clone());
                hv_specific.instance_files.push(err_filename.clone());

                let out = File::create(out_filename)?;
                let err = File::create(err_filename)?;

                cmd.stdout(Stdio::from(out));
                cmd.stderr(Stdio::from(err));

                log::trace!("ROS2 CMD is : {:?}", cmd);
                let child = cmd.spawn()?;
                log::debug!("Child PID {}", child.id());

                hv_specific.pid = child.id();

                instance.hypervisor_specific = Some(serialize_ros2_specific_info(&hv_specific)?);

                if let FDUState::ERROR(_) = instance.status {
                    instance.restarts += 1
                }

                instance.status = FDUState::RUNNING;
                self.connector.local.add_instance(&instance).await?;
                guard
                    .childs
                    .insert(instance_uuid, Arc::new(Mutex::new(child)));
                log::debug!("Instance status {:?}", instance.status);
                guard.fdus.insert(instance_uuid, instance);
                Ok(instance_uuid)
            }
            _ => Err(FError::TransitionNotAllowed),
        }
    }

    async fn run_fdu(&mut self, instance_uuid: Uuid) -> FResult<Uuid> {
        Err(FError::Unimplemented)
    }

    async fn log_fdu(&mut self, instance_uuid: Uuid) -> FResult<String> {
        Err(FError::Unimplemented)
    }
    async fn ls_fdu(&mut self, instance_uuid: Uuid) -> FResult<Vec<String>> {
        Err(FError::Unimplemented)
    }

    async fn file_fdu(&mut self, instance_uuid: Uuid, file_name: String) -> FResult<String> {
        Err(FError::Unimplemented)
    }

    async fn stop_fdu(&mut self, instance_uuid: Uuid) -> FResult<Uuid> {
        log::debug!("Stop instance {:?}", instance_uuid);
        let node_uuid = self.agent.as_ref().unwrap().get_node_uuid().await??;
        let mut instance = self.connector.local.get_instance(instance_uuid).await?;
        log::trace!("Instance status {:?}", instance.status);
        match instance.status {
            FDUState::RUNNING | FDUState::ERROR(_) => {
                let mut guard = self.fdus.write().await;
                instance.status = FDUState::CONFIGURED;

                let mut hv_specific =
                    deserialize_ros2_specific_info(&instance.clone().hypervisor_specific.unwrap())?;

                let c = guard
                    .childs
                    .remove(&instance_uuid)
                    .ok_or(FError::NotFound)?;
                let mut child = c.lock().await;
                log::trace!("Child PID {:?}", child.id());
                if cfg!(feature = "isolation") {
                    log::trace!(
                        "Isolation is active sending kill with sudo to PID {:?}",
                        child.id()
                    );
                    let pid = child.id();
                    kill(Pid::from_raw(pid as i32), Signal::SIGINT)
                        .map_err(|e| FError::HypervisorError(format!("{}", e)))?;
                    log::trace!("Waiting child to exit...");
                    child.wait()?;
                } else {
                    child.kill()?;
                }

                hv_specific.pid = 0;

                instance.hypervisor_specific = Some(serialize_ros2_specific_info(&hv_specific)?);

                self.connector.local.add_instance(&instance).await?;

                log::trace!("Instance status {:?}", instance.status);
                guard.fdus.insert(instance_uuid, instance);

                Ok(instance_uuid)
            }
            _ => Err(FError::TransitionNotAllowed),
        }
    }

    async fn migrate_fdu(&mut self, instance_uuid: Uuid, destination_uuid: Uuid) -> FResult<Uuid> {
        Err(FError::Unimplemented)
    }

    async fn get_fdu_status(&self, instance_uuid: Uuid) -> FResult<FDURecord> {
        let node_uuid = self.agent.as_ref().unwrap().get_node_uuid().await??;
        self.connector.local.get_instance(instance_uuid).await
    }
}

impl ROS2Hypervisor {
    async fn run(&self, stop: async_std::channel::Receiver<()>) -> FResult<()> {
        log::info!("ROS2Hypervisor main loop starting...");

        //starting the Agent-Plugin Server
        let hv_server = self
            .clone()
            .get_hypervisor_plugin_server(self.z.clone(), None);
        let (stopper, _h) = hv_server.connect().await?;
        hv_server.initialize().await?;

        let mut guard = self.fdus.write().await;
        guard.uuid = Some(hv_server.instance_uuid());
        drop(guard);

        self.agent
            .clone()
            .unwrap()
            .register_plugin(
                self.fdus.read().await.uuid.unwrap(),
                PluginKind::HYPERVISOR(String::from("ros2")),
            )
            .await??;

        hv_server.register().await?;

        let (shv, hhv) = hv_server.start().await?;

        let node_uuid = self.agent.as_ref().unwrap().get_node_uuid().await??;
        let mut mon_self = self.clone();
        let monitoring = async {
            loop {
                log::info!("Monitoring loop started");

                // monitoring interveal should be configurable
                task::sleep(Duration::from_secs(mon_self.config.monitoring_interveal)).await;

                let mut local_instances = Vec::new();
                let node_fdus_instances = self.connector.local.get_all_instances().await.unwrap();

                for i in node_fdus_instances {
                    log::trace!("Node FDU: {}", i.uuid);
                    if let Ok(desc) = self.connector.global.get_fdu(i.fdu_uuid).await {
                        if desc.hypervisor == *"bare" {
                            local_instances.push(i)
                        }
                    }
                }

                fn find_process(pid: u32) -> FResult<Process> {
                    let mut processes = processes().unwrap();
                    let find = processes
                        .into_iter()
                        .find(|p| p.as_ref().unwrap().pid() == pid);
                    if let Some(p) = find {
                        match p {
                            Ok(p) => {
                                return Ok(p);
                            }
                            Err(ProcessError::NoSuchProcess { pid }) => {
                                log::error!("Process {} not found", pid);
                                return Err(FError::NotFound);
                            }
                            Err(ProcessError::ZombieProcess { pid }) => {
                                log::error!("Process {} is zombie!", pid);
                                return Err(FError::NotFound);
                            }
                            Err(ProcessError::AccessDenied { pid }) => {
                                log::error!("Access denined for process {}", pid);
                                return Err(FError::NotFound);
                            }
                            Err(ProcessError::PsutilError { pid, source }) => {
                                log::error!("Psutil got error {:?} for PID {}", source, pid);
                                return Err(FError::NotFound);
                            }
                        }
                    }
                    Err(FError::NotFound)
                }

                log::trace!("Local Instances: {:?}", local_instances);

                for mut i in local_instances {
                    if let Some(hv_specific) = i.clone().hypervisor_specific {
                        let mut hv_specific =
                            deserialize_ros2_specific_info(&hv_specific.as_slice()).unwrap();

                        match i.status {
                            FDUState::RUNNING => {
                                log::trace!("State of FDU is expected running");
                                if let Ok(process) = find_process(hv_specific.pid) {
                                    match process.status().unwrap() {
                                        ProcessStatus::Running
                                        | ProcessStatus::Idle
                                        | ProcessStatus::Sleeping => {
                                            log::trace!(
                                                "Process is running, status is coherent..."
                                            );
                                        }
                                        _ => {
                                            log::error!("FDU Instance {} is not running", i.uuid);
                                            // Here we try to recover.
                                            if mon_self.try_restart(i.clone()).await.is_ok() {
                                                log::trace!("FDU re-started correctly");
                                            } else {
                                                log::trace!("Unable to restart FDU {}", i.uuid);
                                            }
                                        }
                                    }
                                } else {
                                    log::trace!(
                                        "Unable to find the process {} for the instance",
                                        hv_specific.pid
                                    );
                                    // Here we try to recover.
                                    if mon_self.try_restart(i.clone()).await.is_ok() {
                                        log::trace!("FDU re-started correctly");
                                    } else {
                                        log::trace!("Unable to restart FDU {}", i.uuid);
                                    }
                                }
                            }
                            FDUState::DEFINED => {
                                log::trace!("State of FDU is expected defined");
                            }
                            FDUState::CONFIGURED => {
                                log::trace!("State of FDU is expected configured");
                            }
                            FDUState::ERROR(e) => {
                                log::error!("State of FDU is error: {}", e);
                            }
                        }
                    } else {
                        log::trace!("FDU {} has no hypervisor specific info", i.uuid);
                    }
                }
            }
        };

        match monitoring.race(stop.recv()).await {
            Ok(_) => log::trace!("Monitoring ending correct"),
            Err(e) => log::trace!("Monitoring ending got error: {}", e),
        }

        self.agent
            .clone()
            .unwrap()
            .unregister_plugin(self.fdus.read().await.uuid.unwrap())
            .await??;

        hv_server.stop(shv).await?;
        hv_server.unregister().await?;
        hv_server.disconnect(stopper).await?;

        log::info!("DummyHypervisor main loop exiting");
        Ok(())
    }

    async fn try_reclean(&mut self, mut instance: FDURecord) -> FResult<Uuid> {
        let node_uuid = self.agent.as_ref().unwrap().get_node_uuid().await??;
        instance.status = FDUState::ERROR("FDU not clean!".to_string());
        self.connector.local.add_instance(&instance).await;

        // then we try to start the fdu.
        Ok(self.clean_fdu(instance.uuid).await?)
    }

    async fn try_reconfigure(&mut self, mut instance: FDURecord) -> FResult<Uuid> {
        let node_uuid = self.agent.as_ref().unwrap().get_node_uuid().await??;
        instance.status = FDUState::ERROR("FDU not configured!".to_string());
        self.connector.local.add_instance(&instance).await;

        // then we try to start the fdu.
        Ok(self.configure_fdu(instance.uuid).await?)
    }

    async fn try_restart(&mut self, mut instance: FDURecord) -> FResult<Uuid> {
        let node_uuid = self.agent.as_ref().unwrap().get_node_uuid().await??;
        instance.status = FDUState::ERROR("FDU not running!".to_string());
        self.connector.local.add_instance(&instance).await;

        // then we try to start the fdu.
        Ok(self.start_fdu(instance.uuid).await?)
    }

    pub async fn start(
        &mut self,
    ) -> (
        async_std::channel::Sender<()>,
        async_std::task::JoinHandle<FResult<()>>,
    ) {
        let local_os = OSClient::find_local_servers(self.z.clone()).await.unwrap();
        if local_os.is_empty() {
            log::error!("Unable to find a local OS interface");
            panic!("No OS Server");
        }

        let local_agent = AgentPluginInterfaceClient::find_local_servers(self.z.clone())
            .await
            .unwrap();
        if local_agent.is_empty() {
            log::error!("Unable to find a local Agent interface");
            panic!("No Agent Server");
        }

        let local_net = NetworkingPluginClient::find_local_servers(self.z.clone())
            .await
            .unwrap();
        if local_net.is_empty() {
            log::error!("Unable to find a local Network plugin interface");
            panic!("No Network Server");
        }

        let os = OSClient::new(self.z.clone(), local_os[0]);
        let agent = AgentPluginInterfaceClient::new(self.z.clone(), local_agent[0]);
        let net = NetworkingPluginClient::new(self.z.clone(), local_net[0]);

        self.agent = Some(agent);
        self.os = Some(os);
        self.net = Some(net);

        // Starting main loop in a task
        let (s, r) = async_std::channel::bounded::<()>(1);
        let plugin = self.clone();
        let h = async_std::task::spawn_blocking(move || {
            async_std::task::block_on(async { plugin.run(r).await })
        });
        (s, h)
    }

    pub async fn stop(&self, stop: async_std::channel::Sender<()>) {
        stop.send(()).await;
    }

    fn get_path(&self) -> Box<std::path::Path> {
        self.config.path.clone()
    }

    fn get_run_path(&self) -> Box<std::path::Path> {
        self.config.run_path.clone()
    }
}

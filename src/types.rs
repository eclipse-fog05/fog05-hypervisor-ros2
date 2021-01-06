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

#![allow(unused_variables)]

extern crate serde;
extern crate serde_json;
extern crate serde_yaml;

use std::collections::HashMap;
use std::process::Child;

use async_std::sync::{Arc, Mutex, RwLock};

use fog05_sdk::agent::{AgentPluginInterfaceClient, OSClient};
use fog05_sdk::fresult::{FError, FResult};
use fog05_sdk::im::fdu::*;
use fog05_sdk::plugins::NetworkingPluginClient;

use serde::{Deserialize, Serialize};
use uuid::Uuid;

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ROS2HVConfig {
    pub pid_file: Box<std::path::Path>,
    pub zlocator: String,
    pub path: Box<std::path::Path>,
    pub run_path: Box<std::path::Path>,
    pub monitoring_interveal: u64,
    pub ros2_distro: String,
    pub rmw: String,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ROS2HVSpecificInfo {
    //pub netns: Uuid,
    pub pid: u32,
    pub env: HashMap<String, String>,
    pub instance_path: String,
    pub instance_files: Vec<String>,
    pub netns: Option<Uuid>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(rename_all = "UPPERCASE")]
pub enum ROS2AppKind {
    RUN,
    LAUNCH,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ROS2HVSpecificDescriptor {
    pub path: Option<String>,
    pub kind: ROS2AppKind,
    pub app_name: String,
    pub entry_point: String,
    pub args: Vec<String>,
    pub env: HashMap<String, String>,
}

#[derive(Clone)]
pub struct ROS2HVState {
    pub fdus: HashMap<Uuid, FDURecord>,
    pub childs: HashMap<Uuid, Arc<Mutex<Child>>>,
    pub uuid: Option<Uuid>,
}

#[derive(Clone)]
pub struct ROS2Hypervisor {
    pub z: Arc<zenoh::net::Session>,
    pub connector: Arc<fog05_sdk::zconnector::ZConnector>,
    pub pid: u32,
    pub config: ROS2HVConfig,
    pub agent: Option<AgentPluginInterfaceClient>,
    pub os: Option<OSClient>,
    pub net: Option<NetworkingPluginClient>,
    pub fdus: Arc<RwLock<ROS2HVState>>,
}

pub fn serialize_ros2_specific_info(data: &ROS2HVSpecificInfo) -> FResult<Vec<u8>> {
    Ok(serde_json::to_string(data)
        .map_err(|e| FError::HypervisorError(format!("{}", e)))?
        .into_bytes())
}

pub fn deserialize_ros2_specific_info(raw_data: &[u8]) -> FResult<ROS2HVSpecificInfo> {
    Ok(serde_json::from_str::<ROS2HVSpecificInfo>(
        std::str::from_utf8(raw_data).map_err(|e| FError::HypervisorError(format!("{}", e)))?,
    )
    .map_err(|e| FError::HypervisorError(format!("{}", e)))?)
}

pub fn serialize_ros2_specific_descriptor(data: &ROS2HVSpecificDescriptor) -> FResult<Vec<u8>> {
    Ok(serde_json::to_string(data)
        .map_err(|e| FError::HypervisorError(format!("{}", e)))?
        .into_bytes())
}

pub fn deserialize_ros2_specific_descriptor(raw_data: &[u8]) -> FResult<ROS2HVSpecificDescriptor> {
    Ok(serde_json::from_str::<ROS2HVSpecificDescriptor>(
        std::str::from_utf8(raw_data).map_err(|e| FError::HypervisorError(format!("{}", e)))?,
    )
    .map_err(|e| FError::HypervisorError(format!("{}", e)))?)
}

pub fn serialize_plugin_config(data: &ROS2HVConfig) -> FResult<Vec<u8>> {
    Ok(serde_yaml::to_string(data)
        .map_err(|e| FError::HypervisorError(format!("{}", e)))?
        .into_bytes())
}

pub fn deserialize_plugin_config(raw_data: &[u8]) -> FResult<ROS2HVConfig> {
    Ok(serde_yaml::from_str::<ROS2HVConfig>(
        std::str::from_utf8(raw_data).map_err(|e| FError::HypervisorError(format!("{}", e)))?,
    )
    .map_err(|e| FError::HypervisorError(format!("{}", e)))?)
}

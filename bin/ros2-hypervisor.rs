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
#![allow(clippy::upper_case_acronyms)]

use std::collections::HashMap;
use std::env;
use std::process;

use async_std::fs;
use async_std::path::Path;
use async_std::prelude::*;
use async_std::sync::{Arc, RwLock};

use zenoh::*;

use fog05_sdk::zconnector::ZConnector;

use async_ctrlc::CtrlC;

use structopt::StructOpt;

use fog05_hypervisor_ros2::types::{deserialize_plugin_config, ROS2HVState, ROS2Hypervisor};

static CONFIG_FILE: &str = "/etc/fos/ros2-hypervisor/config.yaml";

#[derive(StructOpt, Debug)]
struct ROS2HVArgs {
    /// Config file
    #[structopt(short, long, default_value = CONFIG_FILE)]
    config: String,
}

fn am_root() -> bool {
    match env::var("USER") {
        Ok(val) => val == "root",
        Err(_) => false,
    }
}

async fn read_file(path: &Path) -> String {
    fs::read_to_string(path).await.unwrap()
}

#[async_std::main]
async fn main() {
    env_logger::init_from_env(
        env_logger::Env::default().filter_or(env_logger::DEFAULT_FILTER_ENV, "info"),
    );

    let args = ROS2HVArgs::from_args();
    log::info!("Native Hypervisor Plugin -- bootstrap");
    let my_pid = process::id();
    log::info!("PID is {}", my_pid);

    if cfg!(feature = "isolation") && !am_root() {
        log::error!("Isolation require the plugin to run as root!");
        process::exit(-1);
    }

    let conf_file_path = Path::new(&args.config);
    let config =
        deserialize_plugin_config(&(read_file(&conf_file_path).await.into_bytes().as_slice()))
            .unwrap();

    let properties = format!("mode=client;peer={}", config.zlocator.clone());
    let zproperties = Properties::from(properties);
    let z = Arc::new(Zenoh::new(zproperties.clone().into()).await.unwrap());
    let zenoh = Arc::new(zenoh::net::open(zproperties.into()).await.unwrap());
    let zconnector = Arc::new(ZConnector::new(z.clone(), None, None));

    let mut ros2 = ROS2Hypervisor {
        z: zenoh.clone(),
        connector: zconnector.clone(),
        pid: my_pid,
        config,
        agent: None,
        os: None,
        net: None,
        fdus: Arc::new(RwLock::new(ROS2HVState {
            uuid: None,
            fdus: HashMap::new(),
            childs: HashMap::new(),
        })),
    };

    let (s, h) = ros2.start().await;

    //Creating the Ctrl-C handler and racing with agent.run
    let ctrlc = CtrlC::new().expect("Unable to create Ctrl-C handler");
    let mut stream = ctrlc.enumerate().take(1);
    stream.next().await;
    log::trace!("Received Ctrl-C start teardown");

    //Here we send the stop signal to the agent object and waits that it ends
    ros2.stop(s).await;

    //wait for the futures to ends
    h.await.unwrap();

    //zconnector.close();
    //zenoh.close();

    log::info!("Bye!")
}

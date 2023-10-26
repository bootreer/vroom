#[allow(dead_code)]
mod pci;
#[allow(dead_code)]
mod queues;
#[allow(dead_code)]
mod memory;
#[allow(dead_code)]
mod nvme;
#[allow(dead_code)]
mod cmd;

use self::pci::*;
use std::error::Error;

#[allow(unused)]
pub fn init(
    pci_addr: &str
) -> Result<(), Box<dyn Error>> {
    let mut vendor_file = pci_open_resource_ro(pci_addr, "vendor").expect("wrong pci address");
    let mut device_file = pci_open_resource_ro(pci_addr, "device").expect("wrong pci address");
    let mut config_file = pci_open_resource_ro(pci_addr, "config").expect("wrong pci address");

    let vendor_id = read_hex(&mut vendor_file)?;
    let device_id = read_hex(&mut device_file)?;
    let class_id = read_io32(&mut config_file, 8)? >> 24;

    if class_id != 1 {
        return Err(format!("device {} is not a NVMe drive", pci_addr).into());
    }

    // todo: init device

    Ok(())

}

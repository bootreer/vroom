#[allow(dead_code)]
mod cmd;
#[allow(dead_code)]
mod consts;
#[allow(dead_code)]
mod memory;
#[allow(dead_code)]
mod nvme;
#[allow(dead_code)]
mod pci;
#[allow(dead_code)]
mod queues;

use nvme::NvmeDevice;

use self::pci::*;
use std::error::Error;

#[allow(unused)]
pub fn init(pci_addr: &str) -> Result<(), Box<dyn Error>> {
    let mut vendor_file = pci_open_resource_ro(pci_addr, "vendor").expect("wrong pci address");
    let mut device_file = pci_open_resource_ro(pci_addr, "device").expect("wrong pci address");
    let mut config_file = pci_open_resource_ro(pci_addr, "config").expect("wrong pci address");

    let vendor_id = read_hex(&mut vendor_file)?;
    let device_id = read_hex(&mut device_file)?;
    let class_id = read_io32(&mut config_file, 8)? >> 8;

    println!("{:X}", class_id);

    // 0x01 -> mass storage device class id
    // 0x08 -> nvme subclass
    if class_id != 0x0108 {
        return Err(format!("device {} is not a block device", pci_addr).into());
    }

    // todo: init device
    NvmeDevice::init(pci_addr);

    Ok(())
}

/*
#[repr(packed)]
pub struct NvmeRegs {
    /// Controller Capabilities
    cap_low: Mmio<u32>,
    cap_high: Mmio<u32>,
    /// Version
    vs: Mmio<u32>,
    /// Interrupt mask set
    intms: Mmio<u32>,
    /// Interrupt mask clear
    intmc: Mmio<u32>,
    /// Controller configuration
    cc: Mmio<u32>,
    /// Reserved
    _rsvd: Mmio<u32>,
    /// Controller status
    csts: Mmio<u32>,
    /// NVM subsystem reset
    nssr: Mmio<u32>,
    /// Admin queue attributes
    aqa: Mmio<u32>,
    /// Admin submission queue base address
    asq_low: Mmio<u32>,
    asq_high: Mmio<u32>,
    /// Admin completion queue base address
    acq_low: Mmio<u32>,
    acq_high: Mmio<u32>,
    /// Controller memory buffer location
    cmbloc: Mmio<u32>,
    /// Controller memory buffer size
    cmbsz: Mmio<u32>,
}
 */

#[derive(Debug)]
pub struct NvmeNamespace {
    pub id: u32,
    pub blocks: u64,
    pub block_size: u64,
}

#[derive(Debug, Clone, Default)]
pub struct NvmeStats {
    pub completions: u64,
    pub submissions: u64,
}

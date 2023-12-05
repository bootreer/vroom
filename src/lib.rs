#![feature(stdsimd)]
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

#[cfg(target_arch = "aarch64")]
#[inline(always)]
pub(crate) unsafe fn pause() {
    std::arch::aarch64::__yield();
}

#[cfg(target_arch = "x86")]
#[inline(always)]
pub(crate) unsafe fn pause() {
    std::arch::x86::_mm_pause();
}

#[cfg(target_arch = "x86_64")]
#[inline(always)]
pub(crate) unsafe fn pause() {
    std::arch::x86_64::_mm_pause();
}

#[allow(unused)]
pub fn init(pci_addr: &str) -> Result<(), Box<dyn Error>> {
    let mut vendor_file = pci_open_resource_ro(pci_addr, "vendor").expect("wrong pci address");
    let mut device_file = pci_open_resource_ro(pci_addr, "device").expect("wrong pci address");
    let mut config_file = pci_open_resource_ro(pci_addr, "config").expect("wrong pci address");

    let vendor_id = read_hex(&mut vendor_file)?;
    let device_id = read_hex(&mut device_file)?;
    let class_id = read_io32(&mut config_file, 8)? >> 16;
    println!("{:X}", class_id);

    // 0x01 -> mass storage device class id
    // 0x08 -> nvme subclass
    if class_id != 0x0108 {
        return Err(format!("device {} is not a block device", pci_addr).into());
    }

    // todo: init device
    let mut nvme = NvmeDevice::init(pci_addr)?;

    nvme.identify_controller();
    Ok(())
}

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

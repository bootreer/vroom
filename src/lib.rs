#![feature(stdsimd)]

#[allow(unused)]
mod cmd;
#[allow(dead_code)]
mod memory;
mod nvme;
#[allow(dead_code)]
mod pci;
#[allow(dead_code)]
mod queues;

use nvme::NvmeDevice;
use self::pci::*;
use std::error::Error;

use std::time::Instant;

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

pub fn init(pci_addr: &str) -> Result<(), Box<dyn Error>> {
    let mut vendor_file = pci_open_resource_ro(pci_addr, "vendor").expect("wrong pci address");
    let mut device_file = pci_open_resource_ro(pci_addr, "device").expect("wrong pci address");
    let mut config_file = pci_open_resource_ro(pci_addr, "config").expect("wrong pci address");

    let _vendor_id = read_hex(&mut vendor_file)?;
    let _device_id = read_hex(&mut device_file)?;
    let class_id = read_io32(&mut config_file, 8)? >> 16;
    println!("{:X}", class_id);

    // 0x01 -> mass storage device class id
    // 0x08 -> nvme subclass
    if class_id != 0x0108 {
        return Err(format!("device {} is not a block device", pci_addr).into());
    }

    // todo: init device
    let mut nvme = NvmeDevice::init(pci_addr)?;

    nvme.identify_controller()?;
    nvme.create_io_queue_pair()?;
    let ns = nvme.identify_namespace_list(0);

    for n in ns {
        println!("ns_id: {n}");
        nvme.identify_namespace(n);
    }

    // Testing stuff
    let n = 2000;
    let blocks = 8;
    let mut lba = 0;
    let mut read = std::time::Duration::new(0, 0);
    let mut write = std::time::Duration::new(0, 0);
    for _ in 0..n {
        // read
        let before = Instant::now();
        nvme.read(1, blocks, lba);
        read += before.elapsed();
        // println!("{blocks} block read: {:?}", before.elapsed());
        let rand_block = &(0.. (512 * blocks)).map(|_| { rand::random::<u8>() }).collect::<Vec<_>>()[..];


        // write
        let before = Instant::now();
        nvme.write_raw(rand_block, lba)?;
        write += before.elapsed();
        // println!("{blocks} block write: {:?}", before.elapsed());

        lba += blocks as u64;
        // nvme.read(1, 4);
    }

    println!("{blocks} block read: {:?}", read / n);
    println!("{blocks} block write: {:?}", write / n);
    // nvme.test_write("6".repeat(512), 0)?;
    // nvme.test_write("7".repeat(512), 1)?;
    // nvme.test_write("8".repeat(512), 2)?;
    // nvme.test_write("9".repeat(512), 3)?;

    // nvme.read(1, 5);

    Ok(())
}

#[derive(Debug, Clone, Copy)]
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

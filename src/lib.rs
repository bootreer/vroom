#![cfg_attr(target_arch = "aarch64", feature(stdarch_arm_hints))]
#[allow(unused)]
mod cmd;
#[allow(dead_code)]
mod memory;
mod nvme;
#[allow(dead_code)]
mod pci;
#[allow(dead_code)]
mod queues;

use self::pci::*;
use nvme::NvmeDevice;
use std::error::Error;
use std::time::Instant;
// use std::io::Read;
// use std::fs::File;

#[cfg(target_arch = "aarch64")]
#[inline(always)]
pub(crate) fn pause() {
    unsafe {
        std::arch::aarch64::__yield();
    }
}

#[cfg(target_arch = "x86")]
#[inline(always)]
pub(crate) fn pause() {
    unsafe {
        std::arch::x86::_mm_pause();
    }
}

#[cfg(target_arch = "x86_64")]
#[inline(always)]
pub(crate) fn pause() {
    unsafe {
        std::arch::x86_64::_mm_pause();
    }
}

pub fn init(pci_addr: &str) -> Result<(), Box<dyn Error>> {
    let mut vendor_file = pci_open_resource_ro(pci_addr, "vendor").expect("wrong pci address");
    let mut device_file = pci_open_resource_ro(pci_addr, "device").expect("wrong pci address");
    let mut config_file = pci_open_resource_ro(pci_addr, "config").expect("wrong pci address");

    let _vendor_id = read_hex(&mut vendor_file)?;
    let _device_id = read_hex(&mut device_file)?;
    let class_id = read_io32(&mut config_file, 8)? >> 16;

    // 0x01 -> mass storage device class id
    // 0x08 -> nvme subclass
    if class_id != 0x0108 {
        return Err(format!("device {} is not a block device", pci_addr).into());
    }

    let mut nvme = NvmeDevice::init(pci_addr)?;

    nvme.identify_controller()?;
    nvme.create_io_queue_pair()?;
    let ns = nvme.identify_namespace_list(0);

    for n in ns {
        println!("ns_id: {n}");
        nvme.identify_namespace(n);
    }

    // Testing stuff
    let n = 100;
    let n2 = 100;
    let blocks = 1024 * 4;

    let mut read = std::time::Duration::new(0, 0);
    let mut read_batched = std::time::Duration::new(0, 0);
    let mut _write = std::time::Duration::new(0, 0);
    let mut write_batched = std::time::Duration::new(0, 0);

    let mut read_buf = vec![0; blocks * 512];
    let mut read_bbuf = vec![0; blocks * 512];

    for _ in 0..n2 {
        let mut lba = 0;
        for _ in 0..n {
            let rand_block = &(0..(512 * blocks))
                .map(|_| rand::random::<u8>())
                .collect::<Vec<_>>()[..];
            unsafe { (*nvme.buffer.virt)[..].copy_from_slice(rand_block) };

            // write
            // let before = Instant::now();
            // nvme.write_raw(rand_block, lba)?;
            // write += before.elapsed();

            let before = Instant::now();
            nvme.batched_write(1, rand_block, lba, 256)?;
            write_batched += before.elapsed();

            // read
            let before = Instant::now();
            nvme.batched_read(1, &mut read_bbuf[..], lba, 256)?;
            read_batched += before.elapsed();

            let before = Instant::now();
            nvme.read(1, &mut read_buf[..], lba)?;
            read += before.elapsed();
            // assert_eq!(read_buf, rand_block);
            // assert_eq!(read_buf, read_bbuf);

            lba += blocks as u64;
            // nvme.read(1, 4);
        }
    }
    let n = n * n2;

    // println!("{blocks} block write: {:?}", write / n);
    println!("{blocks} block batched write: {:?}", write_batched / n);
    println!("{blocks} block read: {:?}", read / n);
    println!("{blocks} block batched read: {:?}", read_batched / n);
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
    pub reads: u64,
    pub writes: u64,
}

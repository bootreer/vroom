use std::{env, process, thread};
use std::error::Error;
use std::sync::{Arc, Mutex};
use vroom::memory::*;
use vroom::{NvmeDevice, QUEUE_LENGTH};
use rand::{thread_rng, Rng};

pub fn main() -> Result<(), Box<dyn Error>> {
    let mut args = env::args();
    args.next();

    let pci_addr = match args.next() {
        Some(arg) => arg,
        None => {
            eprintln!("Usage: cargo run --example init <pci bus id>");
            process::exit(1);
        }
    };

    let mut nvme = vroom::init(&pci_addr)?;
    nvme.create_io_queue_pair(QUEUE_LENGTH)?;
    let mut nvme = qd32(nvme)?;

    // Testing stuff
    let blocks = 8;
    let bytes = 512 * blocks;
    let ns_blocks = nvme.namespaces.get(&1).unwrap().blocks - blocks - 1;

    let mut buffer: Dma<u8> = Dma::allocate(HUGE_PAGE_SIZE, true)?;

    let n = 100_000;
    let mut read = std::time::Duration::new(0, 0);
    let mut write = std::time::Duration::new(0, 0);

    let mut rng = thread_rng();
    for _ in 0..n {
        let lba = rng.gen_range(0..ns_blocks);
        let rand_block = &(0..bytes).map(|_| rand::random::<u8>()).collect::<Vec<_>>()[..];

        buffer[..rand_block.len()].copy_from_slice(rand_block);

        // write
        let before = std::time::Instant::now();
        nvme.write(&buffer.slice(0..bytes as usize), lba)?;
        write += before.elapsed();

        buffer[..rand_block.len()].fill_with(Default::default);
        let before = std::time::Instant::now();
        nvme.read(&buffer.slice(0..bytes as usize), lba)?;
        read += before.elapsed();

        assert_eq!(&buffer[0..rand_block.len()], rand_block);
        // lba += blocks as u64;
    }

    println!("total completions: {}", nvme.stats.completions);
    println!("total submissions: {}", nvme.stats.submissions);
    println!(
        "read time: {:?}; write time: {:?}; total: {:?}",
        read,
        write,
        read + write
    );

    Ok(())
}

#[allow(unused)]
fn qd32(mut nvme: NvmeDevice) -> Result<NvmeDevice, Box<dyn Error>> {
    let blocks = 8;
    let bytes = 512 * blocks;
    let ns_blocks = Arc::new(nvme.namespaces.get(&1).unwrap().blocks - blocks - 1);

    let nvme = Arc::new(Mutex::new(nvme));
    let mut threads = Vec::new();

    for _ in 0..4 {
        let max_lba = ns_blocks.clone();
        let mut nvme = Arc::clone(&nvme);

        let handle = thread::spawn(move || -> (std::time::Duration, std::time::Duration) {
            let mut rng = rand::thread_rng();
            let seq = &(0..2_500_000).map(|_| rng.gen_range(0..*max_lba)).collect::<Vec<u64>>()[..];

            let blocks = 8;
            let bytes = 512 * blocks;

            let mut read = std::time::Duration::new(0, 0);
            let mut write = std::time::Duration::new(0, 0);
            let mut buffer: Dma<u8> = Dma::allocate(HUGE_PAGE_SIZE, true).unwrap();

            let mut qpair = nvme.lock().unwrap().create_io_queue_pair(32).unwrap();

            // TODO
            for &lba in seq {
                if qpair.sub_queue.is_full() {
                    if let Some(head) = vroom::complete_io(&mut qpair) {
                        qpair.sub_queue.head = head as usize
                    } else {
                        eprintln!("shit"); 
                        continue
                    }
                }

                let rand_block = &(0..bytes).map(|_| rand::random::<u8>()).collect::<Vec<_>>()[..];

                buffer[..rand_block.len()].copy_from_slice(rand_block);

                // write
                let before = std::time::Instant::now();
                // nvme.write(&buffer.slice(0..bytes as usize), lba)?;
                vroom::submit_io(&mut qpair.sub_queue, &buffer.slice(0..rand_block.len()), lba, true);
                write += before.elapsed();

                /* 
                buffer[..rand_block.len()].fill_with(Default::default);
                let before = std::time::Instant::now();
                vroom::submit_io(&mut qpair.sub_queue, &buffer.slice(0..rand_block.len()), lba, false);
                read += before.elapsed();
                */
            }
            (read, write)
        });
        threads.push(handle);
    }

    let (read, write) = threads.into_iter().fold((std::time::Duration::ZERO, std::time::Duration::ZERO), |acc, thread| {
        let res = thread
            .join()
            .expect("The thread creation or execution failed!");
        (acc.0 + res.0, acc.1 + res.1)
    });

    println!(
        "read time: {:?}; write time: {:?}; total: {:?}",
        read,
        write,
        read + write
    );

    // lol
    match Arc::try_unwrap(nvme) {
        Ok(mutex) => {
            match mutex.into_inner() {
                Ok(t) => Ok(t),
                Err(e) => Err(e.into()),
            }
        }
        Err(arc_mutex_t_again) => {
            Err("Arc::try_unwrap failed, not the last reference.".into())
        }
    }
}

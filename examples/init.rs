use std::env;
use std::process;

pub fn main() -> Result<(), Box<dyn std::error::Error>> {
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

    // Testing stuff
    let n = 10;
    let n2 = 100_000;
    let blocks = 8;

    let mut read = std::time::Duration::new(0, 0);
    let mut write = std::time::Duration::new(0, 0);
    let mut read_buf = vec![0; blocks * 512];

    //  let mut write_batched = std::time::Duration::new(0, 0);
    //  let mut read_batched = std::time::Duration::new(0, 0);
    //  let mut read_bbuf = vec![0; blocks * 512];
    let mut rng = rand::thread_rng();
    use rand::seq::SliceRandom;

    let mut seq: Vec<u64> = Vec::from_iter(0..n);

    for _ in 0..n2 {
        seq.shuffle(&mut rng);
        let lba = 0;
        for i in &seq {
            let rand_block = &(0..(512 * blocks))
                .map(|_| rand::random::<u8>())
                .collect::<Vec<_>>()[..];
            unsafe { (*nvme.buffer.virt)[..rand_block.len()].copy_from_slice(rand_block) };

            // write
             let before = std::time::Instant::now();
             nvme.write_raw(rand_block, lba + (*i * blocks as u64))?;
             write += before.elapsed();

            //  let before = Instant::now();
            //  nvme.batched_write(1, rand_block, lba, 256)?;
            //  write_batched += before.elapsed();

            // read
            //  let before = Instant::now();
            //  nvme.batched_read(1, &mut read_bbuf[..], lba, 256)?;
            //  read_batched += before.elapsed();

            let before = std::time::Instant::now();
            nvme.read(1, &mut read_buf[..], lba + (*i * blocks as u64))?;
            read += before.elapsed();

            // assert_eq!(read_buf, rand_block);
            // assert_eq!(read_buf, read_bbuf);

            //  lba += blocks as u64;
        }
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

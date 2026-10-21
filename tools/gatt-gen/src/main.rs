//! `gatt-gen`: read a proto-defined GATT schema, check it, and write the code a
//! target needs.
//!
//! ```sh
//! gatt-gen rust-trouble --out ../calypso-core/src \
//!   --messages crate::pb::msg --server CalypsoGatt ../proto/calypso_gatt.proto
//! gatt-gen --write-options proto      # the vocabulary, for a consumer that wants it
//! ```

use std::path::{Path, PathBuf};
use std::process::ExitCode;

use gatt_gen::generator::{
    client::dart_universal_ble, server::rust_trouble, Config, Target, TARGETS,
};
use gatt_gen::{descriptor, Input};

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match parse(&args) {
        Ok(Command::WriteOptions(dir)) => write_options(&dir),
        Ok(Command::Generate(plan)) => run(&plan),
        Err(problem) => usage(&problem),
    }
}

enum Command {
    WriteOptions(PathBuf),
    Generate(Plan),
}

struct Plan {
    target: Target,
    config: Config,
    /// The folder the generated file goes in: the target names the file.
    out: PathBuf,
    input: Input,
    /// Write the descriptor set here too, if asked.
    fdset: Option<PathBuf>,
    print: bool,
}

fn parse(args: &[String]) -> Result<Command, String> {
    let mut args = args.iter();
    let Some(first) = args.next() else {
        return Err("no target given".to_string());
    };

    if first == "--write-options" {
        let dir = args
            .next()
            .ok_or_else(|| "--write-options needs a directory".to_string())?;
        return Ok(Command::WriteOptions(PathBuf::from(dir)));
    }

    let target = Target::from_name(first).ok_or_else(|| format!("unknown target {first:?}"))?;
    let mut rust = rust_trouble::Config::default();
    let mut dart = dart_universal_ble::Config::default();
    let mut out: Option<PathBuf> = None;
    let mut input = Input::new([] as [String; 0]);
    let mut fdset = None;
    let mut print = false;

    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--out" | "-o" => {
                out = Some(PathBuf::from(
                    args.next()
                        .ok_or_else(|| "--out needs a directory".to_string())?,
                ));
            }
            "--include" | "-I" => {
                input.includes.push(PathBuf::from(
                    args.next()
                        .ok_or_else(|| "--include needs a directory".to_string())?,
                ));
            }
            "--messages" => {
                rust.message_module = args
                    .next()
                    .ok_or_else(|| "--messages needs a module path".to_string())?
                    .clone();
            }
            "--server" => {
                rust.server_name = args
                    .next()
                    .ok_or_else(|| "--server needs a name".to_string())?
                    .clone();
            }
            "--chunk-max" => rust.pipe_chunk_max = number(args.next(), "--chunk-max")?,
            "--pipe-msg-max" => rust.pipe_msg_max = number(args.next(), "--pipe-msg-max")?,
            "--fdset" => {
                fdset = Some(PathBuf::from(
                    args.next()
                        .ok_or_else(|| "--fdset needs a path".to_string())?,
                ));
            }
            "--regenerate-with" => {
                let command = args
                    .next()
                    .ok_or_else(|| "--regenerate-with needs a command".to_string())?
                    .clone();
                rust.regenerate_with = Some(command.clone());
                dart.regenerate_with = Some(command);
            }
            "--dart-messages" => {
                dart.messages_import = args
                    .next()
                    .ok_or_else(|| "--dart-messages needs an import path".to_string())?
                    .clone();
            }
            "--dart-prefix" => {
                dart.messages_prefix = args
                    .next()
                    .ok_or_else(|| "--dart-prefix needs a prefix".to_string())?
                    .clone();
            }
            "--print" => print = true,
            other if other.starts_with('-') => return Err(format!("unknown option {other}")),
            other => input.protos.push(PathBuf::from(other)),
        }
    }

    let out = out.ok_or_else(|| "no --out directory given".to_string())?;
    if input.protos.is_empty() {
        return Err("no .proto files given".to_string());
    }

    let config = match target {
        Target::RustTrouble => Config::RustTrouble(rust),
        Target::DartUniversalBle => Config::DartUniversalBle(dart),
    };

    Ok(Command::Generate(Plan {
        target,
        config,
        out,
        input,
        fdset,
        print,
    }))
}

fn number(value: Option<&String>, option: &str) -> Result<usize, String> {
    value
        .ok_or_else(|| format!("{option} needs a number"))?
        .parse()
        .map_err(|_| format!("{option} needs a number"))
}

fn run(plan: &Plan) -> ExitCode {
    let loaded = match &plan.fdset {
        Some(path) => descriptor::descriptor_set(&plan.input, path)
            .and_then(|()| descriptor::load_from_descriptor_set(path)),
        None => descriptor::load(&plan.input),
    };
    let (gatt, warnings) = match loaded {
        Ok(loaded) => loaded,
        Err(err) => return fail(&err.to_string()),
    };

    for warning in &warnings {
        eprintln!("warning: {warning}");
    }
    if plan.print {
        print!("{gatt}");
    }

    let output = match plan.target.generate(&gatt, &plan.config) {
        Ok(output) => output,
        Err(err) => return fail(&err.to_string()),
    };

    let path = plan.out.join(&output.file_name);
    if let Err(err) = write(&plan.out, &path, &output.source) {
        return fail(&err);
    }
    println!("wrote {}", path.display());
    ExitCode::SUCCESS
}

fn write(dir: &Path, path: &Path, source: &str) -> Result<(), String> {
    std::fs::create_dir_all(dir).map_err(|err| format!("{}: {err}", dir.display()))?;
    std::fs::write(path, source).map_err(|err| format!("{}: {err}", path.display()))
}

fn write_options(dir: &Path) -> ExitCode {
    match gatt_gen::write_options_proto(dir) {
        Ok(path) => {
            println!("wrote {}", path.display());
            ExitCode::SUCCESS
        }
        Err(err) => fail(&err.to_string()),
    }
}

fn fail(message: &str) -> ExitCode {
    eprintln!("error: {message}");
    ExitCode::FAILURE
}

fn usage(problem: &str) -> ExitCode {
    eprintln!("error: {problem}");
    let targets: Vec<&str> = TARGETS.iter().map(|target| target.name()).collect();
    eprintln!("usage: gatt-gen <target> --out DIR [options] <proto>...");
    eprintln!("       gatt-gen --write-options DIR");
    eprintln!();
    eprintln!("targets              {}", targets.join(", "));
    eprintln!("--out                folder for the generated file (the target names the file)");
    eprintln!("--include            a directory the proto files' imports resolve from");
    eprintln!("--fdset              write the descriptor set here as well");
    eprintln!("--print              print the parsed schema");
    eprintln!("--regenerate-with    how the generated file says to remake itself");
    eprintln!();
    eprintln!("rust-trouble         --messages MODULE     where the protobuf types live");
    eprintln!("                     --server NAME         the generated server struct");
    eprintln!("                     --chunk-max N         pipe chunk buffer");
    eprintln!("                     --pipe-msg-max N      largest message a pipe carries");
    eprintln!();
    eprintln!("dart-universal-ble   --dart-messages PATH  import of the protobuf classes");
    eprintln!("                     --dart-prefix NAME     prefix they arrive under");
    eprintln!();
    eprintln!("--write-options      write the schema vocabulary into a directory");
    ExitCode::FAILURE
}

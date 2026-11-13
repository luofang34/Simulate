//! Every `ironbird` command in `README.md` and `book/src/` parses with the real CLI, and the
//! documented Tier 0 commands run as written, so the docs cannot drift from the binary.

#![cfg(not(target_arch = "wasm32"))]

use std::path::{Path, PathBuf};

use ironbird::cli::{CliMode, CliOutcome, execute_cli, parse_args};

/// A documented command: the file it appears in and its arguments after `--`.
#[cfg(test)]
struct DocumentedCommand {
    doc: PathBuf,
    args: Vec<String>,
}

#[cfg(test)]
fn workspace_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..")
}

#[cfg(test)]
fn documented_files() -> Vec<PathBuf> {
    let root = workspace_root();
    let mut files = vec![root.join("README.md")];
    let mut chapters: Vec<PathBuf> = std::fs::read_dir(root.join("book/src"))
        .unwrap()
        .map(|entry| entry.unwrap().path())
        .filter(|path| path.extension().is_some_and(|ext| ext == "md"))
        .collect();
    chapters.sort();
    files.extend(chapters);
    files
}

/// Shell lines of the fenced code blocks in `markdown`, `\` continuations joined and
/// trailing comments dropped.
#[cfg(test)]
fn code_lines(markdown: &str) -> Vec<String> {
    let mut lines = Vec::new();
    let mut in_fence = false;
    let mut pending = String::new();
    for line in markdown.lines() {
        if line.trim_start().starts_with("```") {
            in_fence = !in_fence;
            continue;
        }
        if !in_fence {
            continue;
        }
        let trimmed = line.trim_end();
        pending.push_str(trimmed.trim_end_matches('\\'));
        pending.push(' ');
        if !trimmed.ends_with('\\') {
            let command = pending.split(" #").next().unwrap_or_default().trim();
            lines.push(command.to_owned());
            pending.clear();
        }
    }
    lines
}

/// Arguments after `--` with placeholders made concrete: a `<...>` package directory
/// becomes the built-in C172 package, any other placeholder a plain file name.
#[cfg(test)]
fn concrete_args(tail: &str) -> Vec<String> {
    let mut args: Vec<String> = Vec::new();
    for raw in tail.split_whitespace() {
        let token = raw.trim_matches(|c| c == '\'' || c == '"');
        let concrete = if !token.contains('<') {
            token.to_owned()
        } else if args.last().is_some_and(|flag| flag == "--package-dir") {
            "vehicles/c172s".to_owned()
        } else {
            "placeholder.md".to_owned()
        };
        args.push(concrete);
    }
    args
}

#[cfg(test)]
fn documented_commands() -> Vec<DocumentedCommand> {
    let mut commands = Vec::new();
    for doc in documented_files() {
        let markdown = std::fs::read_to_string(&doc).unwrap();
        for line in code_lines(&markdown) {
            if !line.contains("-p ironbird") {
                continue;
            }
            let Some((_, tail)) = line.split_once(" -- ") else {
                continue;
            };
            commands.push(DocumentedCommand {
                doc: doc.clone(),
                args: concrete_args(tail),
            });
        }
    }
    commands
}

#[test]
fn every_documented_ironbird_command_parses() {
    let commands = documented_commands();
    assert!(
        commands.len() >= 10,
        "found only {} documented ironbird commands; did the extractor break?",
        commands.len()
    );
    for command in &commands {
        if let Err(err) = parse_args(&command.args) {
            panic!(
                "{}: `ironbird {}` is rejected: {err:?}",
                command.doc.display(),
                command.args.join(" ")
            );
        }
    }
}

#[test]
fn documented_tier0_commands_fly_as_written() {
    let root = workspace_root();
    let mut flown = 0_u32;
    for command in documented_commands() {
        let mut args = parse_args(&command.args).unwrap();
        if !matches!(
            args.mode,
            CliMode::Tier0X500 | CliMode::Tier0C172 | CliMode::Tier0Dir
        ) {
            continue;
        }
        args.package_dir = args.package_dir.map(|dir| root.join(dir));
        let outcome = execute_cli(&args).unwrap_or_else(|err| {
            panic!(
                "{}: `ironbird {}` failed: {err:?}",
                command.doc.display(),
                command.args.join(" ")
            )
        });
        let CliOutcome::Rig(summary) = outcome else {
            panic!("Tier 0 mode returned {outcome:?}");
        };
        assert!(
            (summary.simulated_seconds - args.duration_s).abs() < 0.01,
            "{}: flew {} s of a documented {} s",
            command.doc.display(),
            summary.simulated_seconds,
            args.duration_s
        );
        flown = flown.wrapping_add(1);
    }
    assert!(flown >= 3, "only {flown} documented Tier 0 commands found");
}

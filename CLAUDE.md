# NeuroMotion

The rules and guidelines for AI agents are in AGENTS.md, shared by every AI tool;
follow them:

@AGENTS.md

## Claude Code specifics

- Load the full human guides when a task touches their area: @CONTRIBUTING.md for
  process and ground rules, @ARCHITECTURE.md for the code map.
- Use the dedicated tools to read and edit files; run `cargo fmt`,
  `cargo clippy --all-targets -- -D warnings` and `cargo test --release` before
  reporting a task as done.
- Long physics runs: use `--release`; debug builds are many times slower.

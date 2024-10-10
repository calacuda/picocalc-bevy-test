build:
  cargo +stable build

flash:
  elf2uf2-rs -d target/thumbv6m-none-eabi/debug/pico-synth

run:
  DEFMT_LOG=trace cargo +stable run

tmux:
  tmux new -ds pico-synth -n "README"
  tmux send-keys -t pico-synth:README 'nv ./README.md "+set wrap"' ENTER
  @just new-window "Reff" ""
  @just new-window "Edit" ""
  @just new-window "Run" ""
  @just new-window "git" "git status"
  tmux a -t pico-synth

new-window NAME CMD:
  tmux new-w -t pico-synth -n "{{NAME}}"
  tmux send-keys -t pico-synth:"{{NAME}}" "{{CMD}}" ENTER


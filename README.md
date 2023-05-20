# Hagiwo-8-Drum-Sample-playback-with-MIDI

Modified version of the Hagiwo $8 Drum Sample playback module.

https://note.com/solder_state/n/n950ed088c3cb

Reads MIDI CC messages and program changes on channel 1 for control over the sample number, tuning and filter.

* CC 09 controls the tuning of the drum in 128 steps
* CC 10 turns off/on the low pass filter for the drum module
* PGM change 0-47 select the sample to playback.

I moved the trigger input to another pin and added an output to control the micro relay for the low pass filter.

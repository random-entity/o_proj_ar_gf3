s.boot();

// 1. Read stereo audio file
(
~filePath = "tts/test.wav";
~buffer = Buffer.readChannel(s, ~filePath, channels: [0]);
)

// Check audio file
(
{
    PlayBuf.ar(1, ~buffer, BufRateScale.kr(~buffer), loop: 1)
}.play;
)

(
// Define the full processing chain for mono input
SynthDef(\shin, {
    // Busses for each step
    var eqBus, vocoderBus, pitchShiftBus, scrambledBus, finalBus;
	// Don't know why lazy declaration causes syntax error
	var whiteNoise, pulseFreq, pulseWidth, pulseGlitch, carrier, vocoderBands, bandFilters, freq, bandFilter, bandCarrier, vocoder;

    // 1. Input audio from mono WAV file
    var source = PlayBuf.ar(1, ~buffer, BufRateScale.kr(~buffer), loop: 1);  // Mono audio playback

    // 2. EQ with reverse notch and modulated frequency band
    var modFreq = LFSaw.kr(2).range(200, 1000);  // Modulate between 200Hz to 5000Hz with sawtooth
    var eq = BPeakEQ.ar(source, modFreq, 0.1, 20);  // Apply a reverse notch by emphasizing a narrow band

    // Send EQ output to its bus
    eqBus = LocalOut.ar(eq);

	// 3. Vocoder with noise + glitches as carrier, using the EQ output as modulator
	whiteNoise = WhiteNoise.ar(0.75);

    // Generate pulses with frequency randomly changing between 100Hz and 200Hz
    pulseFreq = LFNoise1.kr(4).range(10, 200);  // Randomly change frequency between 100Hz and 200Hz
    pulseWidth = LFDNoise1.kr(4).range(0.1, 0.2);  // Randomly change pulse width between 0.05s and 0.2s
    pulseGlitch = Pulse.ar(pulseFreq, pulseWidth).lag(0.1);  // Pulse with modulated frequency and width, lag for smoothing

	// carrier = whiteNoise;// * pulseGlitch;  // Combine white noise with pulse glitches
	carrier = Saw.ar(440, 0.5);

    vocoderBands = 32;
    bandFilters = Array.fill(vocoderBands, { | i |
        freq = 100 + i * (5000 / (vocoderBands - 1));  // Linearly spaced frequencies
        bandFilter = BPF.ar(source, freq, 1.0);
        bandCarrier = BPF.ar(carrier, freq, 1.0);
        bandFilter * bandCarrier
    });

	vocoder = bandFilters.sum * 0.5;  // Combine the band-pass filtered signals

	// Send the final output to speakers for audition (stereo)
    Out.ar(0, vocoder.dup(2));
}).add;
)

// To run the process, play the Synth:
(
Synth(\shin);
)

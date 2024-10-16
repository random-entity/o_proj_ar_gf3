(
// Function to create a filter bank
~createFilterBank = {
    arg numBands = 16, minFreq = 100, maxFreq = 8000;
    var filters = {
        var result = Array.newClear(numBands); // Create an empty array with the desired size
        numBands.do { |i|
            var freq = minFreq * (maxFreq / minFreq).pow(i / (numBands - 1).asFloat);
            result[i] = BPF.ar(Impulse.ar(0), freq, 0.1); // Fill array with BPF filters
        };
        result
    }.value;
    filters
};

// Function to create envelopes from the filtered signal
~envelopesFromSignal = {
    arg signal, filters;
    filters.collect { |filter|
        var filtered = signal * filter;
        var envelope = EnvGen.kr(Env.perc(0.01, 0.1), Impulse.kr(0.5), doneAction: 2);
        filtered * envelope
    }
};
)

// Vocoder SynthDef
(
SynthDef(\vocoder, {
    arg inBus = 0, carrierBus = 1, numBands = 16;
    var input, carrier, filters, envelopes, modulated;

	// input = SoundIn.ar(inBus);
	input = PlayBuf.ar(1, ~buffer, BufRateScale.kr(~buffer), loop: 1); // Use buffer as input
    carrier = SoundIn.ar(carrierBus);

    filters = ~createFilterBank.value(16);
    envelopes = ~envelopesFromSignal.value(input, filters);

    modulated = envelopes.collect { |env|
        var carrierFiltered = BPF.ar(carrier, 1000, 0.1); // Carrier filter with fixed frequency for demonstration
        carrierFiltered * env
    }.sum;

    Out.ar(0, modulated.dup);
}).add;
)

(
// Load WAV file into buffer
~buffer = Buffer.read(s, "tts/test.wav"); // Update the path to your WAV file
)

// Test the vocoder
(
{
    var input, carrier, vocoder;

    input = SoundIn.ar(0); // Input signal
    carrier = WhiteNoise.ar; // Carrier signal

    vocoder = Synth(\vocoder, [\inBus, 0, \carrierBus, 1]);

    // Play input and carrier signals
    { input }.play;
    { carrier }.play;
}.fork;
)

s.boot

(
var buf = Buffer.read(s, "tts/test.wav");

{
	var in, chain;

	in = PlayBuf.ar(1, buf, BufRateScale.kr(buf), loop: 0);
	chain = FFT(LocalBuf(2048), in);
	chain = PV_MagFreeze(chain);
	IFFT(chain).dup(2);
}.play()
)

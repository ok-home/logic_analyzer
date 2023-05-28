	.file	"fake_hi_interrupt_s.c"
	.text
	.literal_position
	.literal .LC0, la_hi_interrupt_state
	.align	4
	.global	xt_highint5
	.type	xt_highint5, @function
xt_highint5:
	entry	sp, 32
	l32r	a8, .LC0
	l32i.n	a9, a8, 44
	l32i.n	a10, a8, 48
	memw
	s32i.n	a10, a9, 0
	l32i.n	a9, a8, 4
	l32i.n	a10, a8, 8
	memw
	s32i.n	a10, a9, 0
	l32i.n	a9, a8, 28
	l32i.n	a8, a8, 40
	memw
	s32i.n	a8, a9, 0
	retw.n
	.size	xt_highint5, .-xt_highint5
	.ident	"GCC: (crosstool-NG esp-2022r1) 11.2.0"

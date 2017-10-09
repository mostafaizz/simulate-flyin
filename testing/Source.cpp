#include <stdio.h>
#include <smmintrin.h>

int main()
{
	__m128 a, b;
	const int mask = 0x71;

	a.m128_f32[0] = 1.5;
	a.m128_f32[1] = 10.25;
	a.m128_f32[2] = -11.0625;
	a.m128_f32[3] = 81.0;
	b.m128_f32[0] = -1.5;
	b.m128_f32[1] = 3.125;
	b.m128_f32[2] = -50.5;
	b.m128_f32[3] = 100.0;

	__m128 res = _mm_dp_ps(a, b, mask);

	printf_s("Original a: %f\t%f\t%f\t%f\nOriginal b: %f\t%f\t%f\t%f\n",
		a.m128_f32[0], a.m128_f32[1], a.m128_f32[2], a.m128_f32[3],
		b.m128_f32[0], b.m128_f32[1], b.m128_f32[2], b.m128_f32[3]);
	printf_s("Result res: %f\t%f\t%f\t%f\n",
		res.m128_f32[0], res.m128_f32[1], res.m128_f32[2], res.m128_f32[3]);

	return 0;
}
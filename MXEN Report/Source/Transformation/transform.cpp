#include <iostream>
#include <complex>
#include <algorithm>

inline double scalefactor(double a, double b)
{
	if(b > a) std::swap(a, b);
	auto f = [](double x, double y){ return x <= 0.0001 ? 0.0 : std::sqrt((x+y)*(x+y)/(2*x*x)); };
	return f(std::abs(a),std::abs(b));
}

int main(void)
{
	double a, b;
	std::complex<double> ro { sqrt(2.0)/2.0, sqrt(2.0)/2.0 };
	std::cin >> a;
	std::cin >> b;

	std::complex<double> z0 { a, b };
	std::complex<double> z1 = z0 * ro;
	std::complex<double> z2 = z1 * scalefactor(z1.real(), z1.imag());

	std::cout << "rotator: " << ro << '\n'
	          << "input:   " << z0 << '\n'
	          << "rotated: " << z1 << '\n'
			  << "scaled:  " << z2 << '\n';
}


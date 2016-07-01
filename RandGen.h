#ifndef RANDOM_H
#define RANDOM_H
#include <random>
#include <QtGlobal>

/// Random number generator (singleton).
class RandGen
{
public:
	enum RandomType {
		AutoRand,   ///< Decide automatically the best method
		TrueRand,   ///< Try to generate true random number
		PseudoRand,   ///< Pseudo-random
		PseudoUniformRealRand   ///< Pseudo-random uniform distribution in the given floating point range
	};

	static RandGen *instance();

	/// Generate raw random number, depending on the backend.
	quint64 generate( RandomType type = TrueRand );

	/// Generate floating-point random number.
	double generateF( double from = 0.0, double to = 1.0, RandomType type = TrueRand );

private:
	RandGen();
	static RandGen *mInstance;

	std::mt19937 mMersenneTwister;
	std::random_device mRandomDevice;
	std::uniform_real_distribution<double> mUniformReal;
};

#endif // RANDOM_H

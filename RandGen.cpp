#include "RandGen.h"
#include <chrono>

RandGen *RandGen::mInstance = nullptr;

RandGen::RandGen() : mUniformReal(0.0,1.0)
{
	mMersenneTwister.seed(std::chrono::system_clock::now().time_since_epoch().count()+2432);
	if( mRandomDevice.entropy() == 0 )
		{ qWarning( "Random device entropy is zero!" ); }
}
//start_required + ( generator_output *  range_required)/generator_maximum
RandGen *RandGen::instance()
{
	if( mInstance == nullptr )
	{
		mInstance = new RandGen();
	}
	return mInstance;
}

quint64 RandGen::generate(RandGen::RandomType type)
{
	if( type == TrueRand )
	{
		try {
			return mRandomDevice();
		} catch(...)
		{
			qWarning( "Failed to generate tre random, generating pseudo instead..." );
			return generate( PseudoRand );
		}
	}
	else if( type == PseudoUniformRealRand )
	{
		return (quint64)mUniformReal(mMersenneTwister)*2^32;
	}
	else //if( type == PseudoRand || type == AutoRand )
	{
		return mMersenneTwister();
	}
}

double RandGen::generateF(double from, double to, RandGen::RandomType type)
{
	if( type == PseudoRand )
	{
		return from + (double)mMersenneTwister()/mMersenneTwister.max()*(to-from);
	}
	else if( type == TrueRand )
	{
		try {
			return from + (double)mRandomDevice()/mRandomDevice.max()*(to-from);
		} catch(...)
		{
			qWarning( "Failed to generate tre random, generating pseudo instead..." );
			return generateF( from, to, PseudoRand );
		}
	} // AutoRand and PseudoUniformRealRand
	else
	{
		return from + mUniformReal(mMersenneTwister)*(to-from);
	}
}

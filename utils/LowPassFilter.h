//>> [b,a] = cheby2(5, 40, .1)
//b =   0.006214168363272  -0.016289100434138   0.010256559345268 0.010256559345268  -0.016289100434138   0.006214168363272
//a =   1.000000000000000  -4.321771131089053   7.512074874011979 -6.560198653412074   2.876876058046335  -0.506617893008382
//>> [b,a] = cheby2(5, 40, .15)
//b = 0.008907208333976  -0.019493196064054   0.011806637425515  0.011806637425515  -0.019493196064054   0.008907208333976
//a = 1.000000000000000  -3.976811557715253   6.413634357248125  -5.229579435123164   2.152491282008911  -0.357293347027743
//>> [b,a] = cheby2(5, 40, .2)
//b = 0.011825323028793  -0.019461085697466   0.012264284848387  0.012264284848387  -0.019461085697466   0.011825323028793
//a = 1.000000000000000  -3.624706779111254   5.403341592896954 -4.109250496005524   1.588963251720918  -0.249090525141668
// [b,a] = cheby2(5,60,.0667)
//a = { 1.000000000000000e+00, -4.707799276080112e+00, 8.873457978964597e+00, -8.369830340061576e+00, 3.950688626592594e+00, -7.465113011093129e-01};
//b = {4.745556460267435e-04, -1.341869085306422e-03, 8.701575923746904e-04, 8.701575923746898e-04, -1.341869085306421e-03, 4.745556460267429e-04};

#ifndef __LOWPASSFILTER_H__
#define __LOWPASSFILTER_H__

/** A history window vector convolution utility */
class LowPassFilter{
	public:

        /** Given vectors a and b of length order, applies said model to each
         * of nfilters.  Optional valye init_filter_values fills initial x
         * and y values for mx and my */
		LowPassFilter(const double* a, const double* b, int order, int nfilters, const double* init_filter_values = 0);

		~LowPassFilter();

        /** Given vector a vector x with the latest raw values and a vector
         * y to copy the latest results into, writes the newest calculated
         * smooth values into y */
        void Update(const double* x, double* y);

        /** Copies latest computed smooth output my into vector y */
		void Y(double* y);

	private:
		double* ma;
		double* mb;
		int mnfilters;
		int morder;

		double** mx;
		double** my; 	

		int m_initialized_filter_state;

};
#endif


/**************************************************************
WinFilter version 0.8
http://www.winfilter.20m.com
akundert@hotmail.com

Filter type: Band Pass
Filter model: Butterworth
Filter order: 4
Sampling Frequency: 4 KHz
Fc1 and Fc2 Frequencies: 0.121000 KHz and 0.443000 KHz
Coefficents Quantization: float

Z domain Zeros
z = -1.000000 + j 0.000000
z = -1.000000 + j 0.000000
z = -1.000000 + j 0.000000
z = -1.000000 + j 0.000000
z = 1.000000 + j 0.000000
z = 1.000000 + j 0.000000
z = 1.000000 + j 0.000000
z = 1.000000 + j 0.000000

Z domain Poles
z = 0.665763 + j -0.293015
z = 0.665763 + j 0.293015
z = 0.831311 + j -0.173495
z = 0.831311 + j 0.173495
z = 0.939331 + j -0.181486
z = 0.939331 + j 0.181486
z = 0.682480 + j -0.533860
z = 0.682480 + j 0.533860
***************************************************************/
#define NCoef 8
float iir_1_canal(float NewSample) {
//    float ACoef[NCoef+1] = {
//			0.0223693847656250,
//			0,
//			-0.0894927978515625,
//			0,
//			0.134239196777344,
//			0	-0.0894927978515625,
//			0,
//			0.0223693847656250
//    };

//    float BCoef[NCoef+1] = {
//      1,
//			-4.24462890625000,
//			8.41259765625000,
//			-10.3789062500000,
//			8.81738281250000,
//			-5.24609375000000,
//			2.11083984375000,
//			-0.526855468750000,
//			0.0644531250000000
//    };


//  float ACoef[NCoef+1] = {
//        0.02274775833549169000,
//        0.00000000000000000000,
//        -0.09099103334196675900,
//        0.00000000000000000000,
//        0.13648655001295015000,
//        0.00000000000000000000,
//        -0.09099103334196675900,
//        0.00000000000000000000,
//        0.02274775833549169000
//    };

//    float BCoef[NCoef+1] = {
//        1.00000000000000000000,
//        -4.24690542590865050000,
//        8.41960286944806670000,
//        -10.38746377442962800000,
//        8.82320480345669900000,
//        -5.24806890289911450000,
//        2.11078790926885600000,
//        -0.52650368157783511000,
//        0.06440870243726799800
//    };



//    float ACoef[NCoef+1] = {
//        0.00263760754414717800,
//        0.00000000000000000000,
//        -0.01055043017658871200,
//        0.00000000000000000000,
//        0.01582564526488306600,
//        0.00000000000000000000,
//        -0.01055043017658871200,
//        0.00000000000000000000,
//        0.00263760754414717800
//    };

//    float BCoef[NCoef+1] = {
//        1.00000000000000000000,
//        -6.35083265715917110000,
//        17.95777788780421200000,
//        -29.55763204384852100000,
//        30.99370395251691300000,
//        -21.20782480175404000000,
//        9.24885912255414410000,
//        -2.35070834833044230000,
//        0.26671831197300022000
//    };


   float ACoef[NCoef+1] = {
   0.002210597725843,                 0,-0.008842390903372,                 0,
    0.01326358635506,                 0,-0.008842390903372,                 0,
   0.002210597725843
    };

    float BCoef[NCoef+1] = {
                   1,   -6.350413190176,    17.95253043718,   -29.53827103251,
       30.9584137555,   -21.17101493516,    9.226222967527,   -2.342993359724,
     0.2655842872962
    };



    static float y[NCoef+1]; //output samples
    static float x[NCoef+1]; //input samples
    int n;

    //shift the old samples
    for(n=NCoef; n>0; n--) {
       x[n] = x[n-1];
       y[n] = y[n-1];
    }

    //Calculate the new output
    x[0] = NewSample;
    y[0] = ACoef[0] * x[0];
    for(n=1; n<=NCoef; n++)
        y[0] += ACoef[n] * x[n] - BCoef[n] * y[n];
    
    return y[0];
}


/**************************************************************
WinFilter version 0.8
http://www.winfilter.20m.com
akundert@hotmail.com

Filter type: Band Pass
Filter model: Butterworth
Filter order: 4
Sampling Frequency: 400 Hz
Fc1 and Fc2 Frequencies: 10.000000 Hz and 36.000000 Hz
Coefficents Quantization: float

Z domain Zeros
z = -1.000000 + j 0.000000
z = -1.000000 + j 0.000000
z = -1.000000 + j 0.000000
z = -1.000000 + j 0.000000
z = 1.000000 + j 0.000000
z = 1.000000 + j 0.000000
z = 1.000000 + j 0.000000
z = 1.000000 + j 0.000000

Z domain Poles
z = 0.733264 + j -0.255871
z = 0.733264 + j 0.255871
z = 0.863416 + j -0.149657
z = 0.863416 + j 0.149657
z = 0.952641 + j -0.151641
z = 0.952641 + j 0.151641
z = 0.763709 + j -0.455910
z = 0.763709 + j 0.455910
***************************************************************/
#define NCoef 8
float iir_2_canal(float NewSample) {
    float ACoef[NCoef+1] = {
        0.00136303268450570440,
        0.00000000000000000000,
        -0.00545213073802281770,
        0.00000000000000000000,
        0.00817819610703422740,
        0.00000000000000000000,
        -0.00545213073802281770,
        0.00000000000000000000,
        0.00136303268450570440
    };

    float BCoef[NCoef+1] = {
        1.00000000000000000000,
        -6.62605922235520330000,
        19.49710773511294400000,
        -33.28665004635813300000,
        36.07224725631162700000,
        -25.41232534945594400000,
        11.36682967489689500000,
        -2.95205287636485720000,
        0.34094015209810741000
    };

    static float y[NCoef+1]; //output samples
    static float x[NCoef+1]; //input samples
    int n;

    //shift the old samples
    for(n=NCoef; n>0; n--) {
       x[n] = x[n-1];
       y[n] = y[n-1];
    }

    //Calculate the new output
    x[0] = NewSample;
    y[0] = ACoef[0] * x[0];
    for(n=1; n<=NCoef; n++)
        y[0] += ACoef[n] * x[n] - BCoef[n] * y[n];
    
    return y[0];
}










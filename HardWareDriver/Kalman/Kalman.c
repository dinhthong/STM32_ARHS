#include "Kalman.h"
    /* Kalman filter variables */
    float Q_angle= 0.008f; // Process noise variance for the accelerometer
    float Q_bias= 0.003f; // Process noise variance for the gyro bias
    float R_measure= 0.05f;// Measurement noise variance - this is actually the variance of the measurement noise

    float angle = 0.0f;; // The angle calculated by the Kalman filter - part of the 2x1 state vector
    float bias =0.0f;; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    float rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

    float P[2][2]; // Error covariance matrix - This is a 2x2 matrix
		float S;
		float K[2];
		float y;
		 float P00_temp;
		  float P01_temp;
  
    /* Kalman filter variables */
    float Q_anglez= 0.008f;; // Process noise variance for the accelerometer
    float Q_biasz= 0.003f; // Process noise variance for the gyro bias
    float R_measurez= 0.05f; // Measurement noise variance - this is actually the variance of the measurement noise

    float anglez = 0.0f; // The angle calculated by the Kalman filter - part of the 2x1 state vector
    float biasz =0.0f;// The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    float ratez; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

    float Pz[2][2]; // Error covariance matrix - This is a 2x2 matrix
		float Sz;
		float Kz[2];
		float yz;
		 float P00_tempz;
		  float P01_tempz;
float Kalman_getAngle_roll(float newAngle, float newRate, float dt) {
    // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
    // Modified by Kristian Lauszus
    // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
    /* Step 1 */
    rate = newRate - bias;
    angle += dt * rate;

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += (Q_bias) * dt;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
     S = P[0][0] + R_measure; // Estimate error
    /* Step 5 */
     // Kalman gain - This is a 2x1 vector
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
     y = newAngle - angle; // Angle difference
    /* Step 6 */
    angle += K[0] * y;
    bias += K[1] * y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    P00_temp = P[0][0];
    P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
};
//
float Kalman_getAngle_pitch(float newAnglez, float newRatez, float dt) {
    // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
    // Modified by Kristian Lauszus
    // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
    /* Step 1 */
    ratez = newRatez - biasz;
    anglez += dt * ratez;

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    Pz[0][0] += dt * (dt*Pz[1][1] - Pz[0][1] - Pz[1][0] + Q_anglez);
    Pz[0][1] -= dt * Pz[1][1];
    Pz[1][0] -= dt * Pz[1][1];
    Pz[1][1] += (Q_biasz ) * dt;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
     Sz = Pz[0][0] + R_measurez; // Estimate error
    /* Step 5 */
     // Kalman gain - This is a 2x1 vector
    Kz[0] = Pz[0][0] / Sz;
    Kz[1] = Pz[1][0] / Sz;

    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
     yz = newAnglez - anglez; // Angle difference
    /* Step 6 */
    anglez += Kz[0] * yz;
    biasz += Kz[1] * yz;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    P00_tempz = Pz[0][0];
    P01_tempz = Pz[0][1];

    Pz[0][0] -= Kz[0] * P00_tempz;
    Pz[0][1] -= Kz[0] * P01_tempz;
    Pz[1][0] -= Kz[1] * P00_tempz;
    Pz[1][1] -= Kz[1] * P01_tempz;

    return anglez;
};

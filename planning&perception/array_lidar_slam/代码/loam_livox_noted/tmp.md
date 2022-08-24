Computes the error term for two poses that have a relative pose measurement
    between them. Let the hat variables be the measurement. We have two poses x_a
    and x_b. Through sensor measurements we can measure the transformation of
    frame B w.r.t frame A denoted as t_ab_hat. We can compute an error metric
    between the current estimate of the poses and the measurement.
    
    In this formulation, we have chosen to represent the rigid transformation as
    a Hamiltonian quaternion, q, and position, p. The quaternion ordering is
    [x, y, z, w].

    The estimated measurement is:
         t_ab = [ p_ab ]  = [ R(q_a)^T * (p_b - p_a) ]
                [ q_ab ]    [ q_a^{-1] * q_b         ]
    
    where ^{-1} denotes the inverse and R(q) is the rotation matrix for the
    quaternion. Now we can compute an error metric between the estimated and
    measurement transformation. For the orientation error, we will use the
    standard multiplicative error resulting in:
    
      error = [ p_ab - \hat{p}_ab                 ]
              [ 2.0 * Vec(q_ab * \hat{q}_ab^{-1}) ]
    
    where Vec(*) returns the vector (imaginary) part of the quaternion. Since
    the measurement has an uncertainty associated with how accurate it is, we
    will weight the errors by the square root of the measurement information
    matrix:
    
      residuals = I^{1/2) * error
    where I is the information matrix which is the inverse of the covariance.
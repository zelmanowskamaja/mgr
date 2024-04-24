
#include <optional>


#include <kalman/ExtendedKalmanFilter.hpp>
#include <kalman/LinearizedSystemModel.hpp>



template<typename T>
class State : public Kalman::Vector<T, 4>
{
public:
    KALMAN_VECTOR(State, T, 4)
  
    T x()       const { return (*this)[ 0 ]; }
    T y()       const { return (*this)[ 1 ]; }
    T theta()   const { return (*this)[ 2 ]; }
    T phi()     const { return (*this)[ 3 ]; }
    
    T& x()      { return (*this)[ 0 ]; }
    T& y()      { return (*this)[ 1 ]; }
    T& theta()  { return (*this)[ 2 ]; }
    T& phi()    { return (*this)[ 3 ]; }
};

template<typename T>
class Control : public Kalman::Vector<T, 2>
{
public:
    KALMAN_VECTOR(Control, T, 2)
    T du1()       const { return (*this)[ 0 ]; }
    T du2()  const { return (*this)[1 ]; }
    
    T& du1()      { return (*this)[ 0 ]; }
    T& du2() { return (*this)[ 1 ]; }
};

template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class SystemModel : public Kalman::LinearizedSystemModel<State<T>, Control<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
	typedef State<T> S;
    
    //! Control type shortcut definition
    typedef Control<T> C;
    
    /**
     * @brief Definition of (non-linear) state transition function
     *
     * This function defines how the system state is propagated through time,
     * i.e. it defines in which state \f$\hat{x}_{k+1}\f$ is system is expected to 
     * be in time-step \f$k+1\f$ given the current state \f$x_k\f$ in step \f$k\f$ and
     * the system control input \f$u\f$.
     *
     * @param [in] x The system state in current time-step
     * @param [in] u The control vector input
     * @returns The (predicted) system state in the next time-step
     */
    S f(const S& x, const C& u) const
    {
        S x_;
        auto newOrientation_phi = x.phi() + u.du2();
        auto newOrientation_theta = x.theta() + 1/2.5*std::tan(newOrientation_phi)*u.du1();
        
        //remember to divide inputs by dt
        x_.x() = x.x() + std::cos(newOrientation_theta)*u.du1();
        x_.y() = x.y() + std::sin(newOrientation_theta)*u.du1();
        x_.theta() = newOrientation_theta;
        x_.phi() = newOrientation_phi;
      
        return x_;
    }
    
protected:
    /**
     * @brief Update jacobian matrices for the system state transition function using current state
     *
     * This will re-compute the (state-dependent) elements of the jacobian matrices
     * to linearize the non-linear state transition function \f$f(x,u)\f$ around the
     * current state \f$x\f$.
     *
     * @note This is only needed when implementing a LinearizedSystemModel,
     *       for usage with an ExtendedKalmanFilter or SquareRootExtendedKalmanFilter.
     *       When using a fully non-linear filter such as the UnscentedKalmanFilter
     *       or its square-root form then this is not needed.
     *
     * @param x The current system state around which to linearize
     * @param u The current system control input
     */
    void updateJacobians( const S& x, const C& u )
    {
        // F = df/dx (Jacobian of state transition w.r.t. the state)
        this->F.setIdentity();
        
        // W = df/dw (Jacobian of state transition w.r.t. the noise)
        this->W.setIdentity();
        // TODO: more sophisticated noise modelling
        //       i.e. The noise affects the the direction in which we move as 
        //       well as the velocity (i.e. the distance we move)
    }
};

template<typename T>
class VelocityMeasurement : public Kalman::Vector<T, 2>
{
public:
    KALMAN_VECTOR(VelocityMeasurement, T, 2)

    T u1()  const { return (*this)[ 0]; }
    T u2()  const { return (*this)[ 1]; }
    T& u1() { return (*this)[ 0 ]; }
    T& u2() { return (*this)[ 1 ]; }
};

template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class VelocityMeasurementModel : public Kalman::LinearizedMeasurementModel<Control<T>, VelocityMeasurement<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
    typedef Control<T> C;
    
    //! Measurement type shortcut definition
    typedef VelocityMeasurement<T> M;
    
    VelocityMeasurementModel()
    {
        // Setup jacobians. As these are static, we can define them once
        // and do not need to update them dynamically
        this->H.setIdentity();
        this->V.setIdentity();
    }
    
    /**
     * @brief Definition of (possibly non-linear) measurement function
     *
     * This function maps the system state to the measurement that is expected
     * to be received from the sensor assuming the system is currently in the
     * estimated state.
     *
     * @param [in] x The system state in current time-step
     * @returns The (predicted) sensor measurement for the system state
     */
    M h(const C& x) const override
    {
        M measurement;
        
        // Measurement is given by the actual robot orientation
        measurement.u1() = x.du1();
        measurement.u2() = x.du2();
        
        return measurement;
    }
};


/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 * Based on the original GTSAM BetweenFactor
 *
 * The GTSAM BetweenFactor does:
 *
 * Key1 = Key2 * Measurement
 *
 * The Inbetween factor does:
 *
 * Key1 = Measurement * Key2
 */
#ifndef TAGSLAM_INBETWEEN_FACTOR_H
#define TAGSLAM_INBETWEEN_FACTOR_H

#include <ostream>

#include <gtsam/base/Testable.h>
#include <gtsam/base/Lie.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace tagslam {

  template<class VALUE>
  class InBetweenFactor: public gtsam::NoiseModelFactor2<VALUE, VALUE> {
    BOOST_CONCEPT_ASSERT((gtsam::IsTestable<VALUE>));
    BOOST_CONCEPT_ASSERT((gtsam::IsLieGroup<VALUE>));

  public:

    typedef VALUE T;

  private:

    typedef InBetweenFactor<VALUE> This;
    typedef gtsam::NoiseModelFactor2<VALUE, VALUE> Base;

    VALUE measured_; /** The measurement */

  public:

    // shorthand for a smart pointer to a factor
    typedef typename boost::shared_ptr<InBetweenFactor> shared_ptr;

    /** default constructor - only use for serialization */
    InBetweenFactor() {}

    /** Constructor */
    InBetweenFactor(gtsam::Key key1, gtsam::Key key2, const VALUE& measured,
                    const gtsam::SharedNoiseModel& model) :
      Base(model, key1, key2), measured_(measured) {
    }

    virtual ~InBetweenFactor() {}

    /// @return a deep copy of this factor
    virtual gtsam::NonlinearFactor::shared_ptr clone() const {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
          gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

    /** implement functions needed for Testable */

    /** print */
    virtual void print(const std::string& s, const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const {
      std::cout << s << "InBetweenFactor("
          << keyFormatter(this->key1()) << ","
          << keyFormatter(this->key2()) << ")\n";
      gtsam::traits<T>::Print(measured_, "  measured: ");
      this->noiseModel_->print("  noise model: ");
    }

    /** equals */
    virtual bool equals(const gtsam::NonlinearFactor& expected, double tol=1e-9) const {
      const This *e =  dynamic_cast<const This*> (&expected);
      return e != NULL && Base::equals(*e, tol) && gtsam::traits<T>::Equals(this->measured_, e->measured_, tol);
    }

    gtsam::Vector evaluateError(const T& p1, const T& p2, boost::optional<gtsam::Matrix&> H1 =
                                boost::none, boost::optional<gtsam::Matrix&> H2 = boost::none) const {
      // hx = p1^-1 * p2
      T hx = gtsam::traits<T>::Between(p2.inverse(), p1.inverse(), H1, H2); // h(x)
      // manifold equivalent of h(x)-z -> log(z,h(x))
      return gtsam::traits<T>::Local(hx, measured_);
    }

    /** return the measured */
    const VALUE& measured() const {
      return measured_;
    }

    /** number of variables attached to this factor */
    std::size_t size() const {
      return 2;
    }

  private:

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & boost::serialization::make_nvp("NoiseModelFactor2",
          boost::serialization::base_object<Base>(*this));
      ar & BOOST_SERIALIZATION_NVP(measured_);
    }
  }; // \class InBetweenFactor

}
#endif

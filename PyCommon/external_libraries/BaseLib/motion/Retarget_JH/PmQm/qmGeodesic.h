
#ifndef __QM_GEODESIC_H
#define __QM_GEODESIC_H

class QmGeodesic
{
  private:
	jhm::quater		orientation;
	jhm::unit_vector	axis;

  public:
	//
	// Constructors
	//
	QmGeodesic() {}
	QmGeodesic( jhm::quater const& q, jhm::unit_vector const& v )
		{ orientation=q; axis=v; }

	//
	// Evaluate points on geodesic
	//
	jhm::quater	getValue( m_real );

	//
	// Set and get parameters
	//
    void        setOrientation( jhm::quater const& q ) { orientation=q; }
    void        setAxis( jhm::unit_vector const& v )   { axis=v; }
    jhm::quater      getOrientation() { return orientation; }
    jhm::unit_vector getAxis()        { return axis; }

	//
	// Metric functions
	//
	jhm::quater nearest( jhm::quater const&, m_real& );
	jhm::quater farthest( jhm::quater const&, m_real& );
	
	jhm::quater nearest( jhm::quater const& );
	jhm::quater farthest( jhm::quater const& );
};

jhm::transf PlaneProject( jhm::transf const& );

#endif


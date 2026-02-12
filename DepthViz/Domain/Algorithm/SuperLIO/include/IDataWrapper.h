#ifndef LI2Sup_IDATAWRAPPER_H
#define LI2Sup_IDATAWRAPPER_H

#include <memory>
#include "common/ds.h"
#include "lio/ESKF.h"

namespace LI2Sup {

class IDataWrapper {
public:
    using Ptr = std::shared_ptr<IDataWrapper>;
    
    virtual ~IDataWrapper() = default;

    virtual bool sync_measure(MeasureGroup& meas) = 0;
    virtual void setESKF(ESKF::Ptr& eskf) = 0;
    
    virtual void pub_odom(const NavState& state) = 0;
    virtual void pub_cloud_world(const BASIC::CloudPtr& pc, double time) = 0;
    virtual void pub_cloud2planner(const BASIC::CloudPtr& pc, double time) = 0;
    
    // Add other methods used by SuperLIO if any
};

} // namespace LI2Sup

#endif // LI2Sup_IDATAWRAPPER_H

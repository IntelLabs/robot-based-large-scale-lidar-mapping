
#ifndef GZ_SIM_SYSTEMS_PTUCOLLISION_HH_
#define GZ_SIM_SYSTEMS_PTUCOLLISION_HH_

#include <memory>
#include <gz/sim/System.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  // Forward declaration
  class PtuCollisionPrivate;

  class PtuCollision
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate,
        public ISystemPostUpdate,
        public ISystemReset
  {
    /// \brief Constructor
    public: PtuCollision();

    /// \brief Destructor
    public: ~PtuCollision() override = default;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    /// Documentation inherited
    public: void PreUpdate(const UpdateInfo &_info,
                           EntityComponentManager &_ecm) final;

    // Documentation inherited
    public: void Reset(const gz::sim::UpdateInfo &_info,
                       gz::sim::EntityComponentManager &_ecm) final;

    // Documentation inherited
    public: void PostUpdate(
                const gz::sim::UpdateInfo &_info,
                const gz::sim::EntityComponentManager &_ecm) override;

    /// \brief Private data pointer
    private: std::unique_ptr<PtuCollisionPrivate> dataPtr;
  };
  }
}
}
}

#endif
#include "drake/systems/lcm/lcm_publisher_system.h"

#include <cstdint>
#include <utility>
#include <vector>

#include "drake/common/text_logging.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcm/drake_lcm_interface.h"

namespace drake {
namespace systems {
namespace lcm {

using drake::lcm::DrakeLcmInterface;
using drake::lcm::DrakeLcm;

// Template Specialization requires these two functions to be declared before
// their usage.

// Takes the VectorBase from the input port of the context and publishes
// it onto an LCM channel. This function is called automatically, as
// necessary, at the requisite publishing period (if a positive publish period
// was passed to the constructor) or per a simulation step (if no publish
// period or publish period = 0.0 was passed to the constructor).
template <>
EventStatus LcmPublisherTemplatedSystem<double>::PublishInputAsLcmMessage(
    const Context<double>& context) const {
  SPDLOG_TRACE(drake::log(), "Publishing LCM {} message", channel_);
  DRAKE_ASSERT(serializer_ != nullptr);

  // Converts the input into LCM message bytes.
  const AbstractValue& input = get_input_port().Eval<AbstractValue>(context);
  std::vector<uint8_t> message_bytes;
  serializer_->Serialize(input, &message_bytes);

  // Publishes onto the specified LCM channel.
  lcm_->Publish(channel_, message_bytes.data(), message_bytes.size(),
                context.get_time());

  return EventStatus::Succeeded();
}


template <>
std::string LcmPublisherTemplatedSystem<double>::make_name(const std::string& channel) {
  return "LcmPublisherSystem(" + channel + ")";
}

template <>
LcmPublisherTemplatedSystem<double>::LcmPublisherTemplatedSystem(
    const std::string& channel,
    std::unique_ptr<SerializerInterface> serializer,
    DrakeLcmInterface* lcm,
    const TriggerTypeSet& publish_triggers,
    double publish_period)
    : systems::LeafSystem<double>(systems::SystemTypeTag<LcmPublisherTemplatedSystem>{}), channel_(channel),
      serializer_(std::move(serializer)),
      owned_lcm_(lcm ? nullptr : new DrakeLcm()),
      lcm_(lcm ? lcm : owned_lcm_.get()) {
  DRAKE_DEMAND(serializer_ != nullptr);
  DRAKE_DEMAND(lcm_);
  DRAKE_DEMAND(publish_period >= 0.0);
  DRAKE_DEMAND(!publish_triggers.empty());

  // Check that publish_triggers does not contain an unsupported trigger.
  for (const auto& trigger : publish_triggers) {
      DRAKE_THROW_UNLESS((trigger == TriggerType::kForced) ||
        (trigger == TriggerType::kPeriodic) ||
        (trigger == TriggerType::kPerStep));
  }

  // Declare a forced publish so that any time Publish(.) is called on this
  // system (or a Diagram containing it), a message is emitted.
  if (publish_triggers.find(TriggerType::kForced) != publish_triggers.end()) {
    this->DeclareForcedPublishEvent(
      &LcmPublisherTemplatedSystem<double>::PublishInputAsLcmMessage);
  }

  DeclareAbstractInputPort("lcm_message", *serializer_->CreateDefaultValue());

  set_name(make_name(channel_));
  if (publish_triggers.find(TriggerType::kPeriodic) != publish_triggers.end()) {
    DRAKE_THROW_UNLESS(publish_period > 0.0);
    const double offset = 0.0;
    this->DeclarePeriodicPublishEvent(
        publish_period, offset,
        &LcmPublisherTemplatedSystem<double>::PublishInputAsLcmMessage);
  } else {
    // publish_period > 0 without TriggerType::kPeriodic has no meaning and is
    // likely a mistake.
    DRAKE_THROW_UNLESS(publish_period == 0.0);
  }

  if (publish_triggers.find(TriggerType::kPerStep) != publish_triggers.end()) {
    this->DeclarePerStepEvent(
    systems::PublishEvent<double>([this](
        const systems::Context<double>& context,
        const systems::PublishEvent<double>&) {
      // TODO(edrumwri) Remove this code once set_publish_period(.) has
      // been removed; it exists so that one does not get both a per-step
      // publish and a periodic publish if a user constructs the publisher
      // the "old" way (construction followed by set_publish_period()).
      if (this->disable_internal_per_step_publish_events_)
        return;

      this->PublishInputAsLcmMessage(context);
    }));
  }
}

template <typename T>
LcmPublisherTemplatedSystem<T>::LcmPublisherTemplatedSystem(
    const std::string& channel,
    std::unique_ptr<SerializerInterface> serializer,
    DrakeLcmInterface* lcm, double publish_period)
    : LcmPublisherTemplatedSystem(channel, std::move(serializer), lcm,
      (publish_period > 0.0) ?
      TriggerTypeSet({TriggerType::kForced, TriggerType::kPeriodic}) :
      TriggerTypeSet({TriggerType::kForced, TriggerType::kPerStep}),
      publish_period) {}




template <typename T>
template <typename U>
LcmPublisherTemplatedSystem<T>::LcmPublisherTemplatedSystem(const LcmPublisherTemplatedSystem<U>& other) :
      LcmPublisherTemplatedSystem("",
      nullptr,
      nullptr) {
        auto value = other.serializer_->CreateDefaultValue();
        this->DeclareAbstractInputPort("lcm_message", *value);
      }

template<typename T>
LcmPublisherTemplatedSystem<T>::~LcmPublisherTemplatedSystem() {}

template<>
void LcmPublisherTemplatedSystem<double>::AddInitializationMessage(
    InitializationPublisher initialization_publisher) {
  DRAKE_DEMAND(!!initialization_publisher);

  initialization_publisher_ = std::move(initialization_publisher);

  DeclareInitializationEvent(systems::PublishEvent<double>(
      systems::TriggerType::kInitialization,
      [this](const systems::Context<double>& context,
             const systems::PublishEvent<double>&) {
        this->initialization_publisher_(context, this->lcm_);
      }));
}


template <>
const std::string& LcmPublisherTemplatedSystem<double>::get_channel_name() const {
  return channel_;
}

template <>
drake::lcm::DrakeLcmInterface& LcmPublisherTemplatedSystem<double>::lcm()
{
  DRAKE_DEMAND(lcm_ != nullptr);
  return *lcm_;
}

template <typename T>
LcmPublisherTemplatedSystem<T>::LcmPublisherTemplatedSystem(
    const std::string& channel,
    std::unique_ptr<SerializerInterface>,
    DrakeLcmInterface*,
    const TriggerTypeSet&,
    double): systems::LeafSystem<T>(systems::SystemTypeTag<LcmPublisherTemplatedSystem>{}),
    channel_(channel), serializer_(nullptr), owned_lcm_(nullptr), lcm_(nullptr) 
    {  
    }

template<typename T>
void LcmPublisherTemplatedSystem<T>::AddInitializationMessage(
    InitializationPublisher) {
  throw std::runtime_error("LcmPublisherTemplatedSystem<T> is only active for scalar type double.");
}

template <typename T>
EventStatus LcmPublisherTemplatedSystem<T>::PublishInputAsLcmMessage(
    const Context<T>&) const {
  throw std::runtime_error("LcmPublisherTemplatedSystem<T> is only active for scalar type double.");
  return EventStatus::Succeeded();
}

template <typename T>
const std::string& LcmPublisherTemplatedSystem<T>::get_channel_name() const {
  throw std::runtime_error("LcmPublisherTemplatedSystem<T> is only active for scalar type double.");
  return channel_;
}

template <typename T>
std::string LcmPublisherTemplatedSystem<T>::make_name(const std::string& channel) {
  throw std::runtime_error("LcmPublisherTemplatedSystem<T> is only active for scalar type double.");
  return "LcmPublisherSystem(" + channel + ")";
}

}  // namespace lcm
}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::systems::lcm::LcmPublisherTemplatedSystem)

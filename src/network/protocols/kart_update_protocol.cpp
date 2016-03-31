#include "network/protocols/kart_update_protocol.hpp"

#include "karts/abstract_kart.hpp"
#include "karts/controller/controller.hpp"
#include "modes/world.hpp"
#include "network/event.hpp"
#include "network/network_config.hpp"
#include "network/protocol_manager.hpp"
#include "utils/time.hpp"

KartUpdateProtocol::KartUpdateProtocol() : Protocol(PROTOCOL_KART_UPDATE)
{
    m_next_time     = 0;
    m_previous_time = -1;
    // Allocate arrays to store one position and rotation for each kart
    // (which is the update information from the server to the client).
    m_next_positions.resize(World::getWorld()->getNumKarts());
    m_next_quaternions.resize(World::getWorld()->getNumKarts());
    m_previous_positions.resize(World::getWorld()->getNumKarts());
    m_previous_quaternions.resize(World::getWorld()->getNumKarts());

    // This flag keeps track if valid data for an update is in
    // the arrays
    m_was_updated = false;
}   // KartUpdateProtocol

// ----------------------------------------------------------------------------
KartUpdateProtocol::~KartUpdateProtocol()
{
}   // ~KartUpdateProtocol

// ----------------------------------------------------------------------------
void KartUpdateProtocol::setup()
{
}   // setup

// ----------------------------------------------------------------------------
/** Store the update events in the queue. Since the events are handled in the
 *  synchronous notify function, there is no lock necessary to 
 */
bool KartUpdateProtocol::notifyEvent(Event* event)
{
    // It might be possible that we still receive messages after
    // the game was exited, so make sure we still have a world.
    if (event->getType() != EVENT_TYPE_MESSAGE || !World::getWorld())
        return true;
    NetworkString &ns = event->data();
    if (ns.size() < 33)
    {
        Log::info("KartUpdateProtocol", "Message too short.");
        return true;
    }
    // Save the previous data that was applied
    float my_time = World::getWorld()->getTime();
    float next_time = ns.getFloat();  // get the time from the message

    // Save the current 'next' data as previous data, but only if this next
    // data is before the current time on the client. This way we always have
    // previous_time < my_time < next_time
    // (short of network hickups causing next_time to be smaller than my_time,
    // which should resolve itself once all in-transit packages have been 
    // processed, since server time will always be ahead of client time.
    bool save_current_as_previous = my_time > m_next_time;
    if(save_current_as_previous )
        m_previous_time = m_next_time;
    m_next_time = next_time;
    while(ns.size() >= 29)
    {
        uint8_t kart_id                 = ns.getUInt8();
        Vec3 xyz                        = ns.getVec3();
        btQuaternion quat               = ns.getQuat();
        if(save_current_as_previous)
        {
            m_previous_positions[kart_id] = m_next_positions[kart_id];
            m_previous_quaternions[kart_id] = m_next_quaternions[kart_id];
        }
        m_next_positions[kart_id] = xyz;
        m_next_quaternions[kart_id]     = quat;
    }   // while ns.size()>29

    // Set the flag that a new update was received
    m_was_updated = true;
    return true;
}   // notifyEvent

// ----------------------------------------------------------------------------
/** Sends regular update events from the server to all clients and from the
 *  clients to the server (FIXME - is that actually necessary??)
 *  Then it applies all update events that have been received in notifyEvent.
 *  This two-part implementation means that if the server should send two
 *  or more updates before this client handles them, only the last one will
 *  actually be handled (i.e. outdated kart position updates are discarded).
 *  \param dt Time step size (used for interpolation).
 */
void KartUpdateProtocol::update(float dt)
{
    if (!World::getWorld())
        return;
    static double time = 0;
    double current_time = StkTime::getRealTime();
    // Dumb clients need updates as often as possible.
    // Otherwise update 10 times a second only
    if (NetworkConfig::get()->isServer() )
    {
//        if( NetworkConfig::get()->useDumbClient() ||
        if (current_time > time + 0.1               )
        {
            time = current_time;
            sendKartUpdates();
        }   // if (current_time > time + 0.1)
        return;
    }

    // Now handle all update events that have been received on a client.
    // There is no lock necessary, since receiving new positions is done in
    // notifyEvent, which is called from the same thread that calls this
    // function.
    if (m_next_time != m_previous_time)
    {

        // no update was received, interpolate
        for (unsigned id = 0; id < m_next_positions.size(); id++)
        {
            AbstractKart *kart = World::getWorld()->getKart(id);
            float adjust_time = World::getWorld()->getTime() - m_next_time;

            float dt_server = m_next_time - m_previous_time;
            float f = (World::getWorld()->getTime() - m_previous_time)
                    / (m_next_time                  - m_previous_time);
            Vec3 xyz = m_previous_positions[id]
                     +  (m_next_positions[id] - m_previous_positions[id])*f;
#ifdef LOG_POSITION_AND_TIME
            Log::error("xyz", "%f %f %f %f  y %f %f %f f %f",
                World::getWorld()->getTime(), m_previous_time, m_next_time,
                dt,
                kart->getXYZ().getY(),
                m_previous_positions[id].getY(),
                m_next_positions[id].getY(),
                f);
#endif
            kart->setXYZ(xyz);
            btQuaternion q =
                m_previous_quaternions[id].slerp(m_next_quaternions[id], f);
            kart->setRotation(q);
        }   // for id in m_next_position
    }   // no update was received, interpolate
    else
        Log::error("xyz", "identical %f %f", m_next_time, m_previous_time);
           

    m_was_updated = false;
}   // update

// ----------------------------------------------------------------------------
void KartUpdateProtocol::sendKartUpdates()
{
    World *world = World::getWorld();
    NetworkString *ns = getNetworkString(4 + world->getNumKarts() * 29);
    ns->setSynchronous(true);
    ns->addFloat(world->getTime());
    for (unsigned int i = 0; i < world->getNumKarts(); i++)
    {
        AbstractKart* kart = world->getKart(i);
        Vec3 xyz = kart->getXYZ();
        ns->addUInt8(kart->getWorldKartId());
        ns->add(xyz).add(kart->getRotation());
        Log::verbose("KartUpdateProtocol",
                     "Sending %d's positions %f %f %f",
                     kart->getWorldKartId(), xyz[0], xyz[1], xyz[2]);
    }
    sendMessageToPeersChangingToken(ns, /*reliable*/false);
    delete ns;
}   // sendKartUpdates

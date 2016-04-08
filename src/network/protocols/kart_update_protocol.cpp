#include "network/protocols/kart_update_protocol.hpp"

#include "karts/abstract_kart.hpp"
#include "karts/controller/controller.hpp"
#include "modes/world.hpp"
#include "network/event.hpp"
#include "network/network_config.hpp"
#include "network/protocol_manager.hpp"
#include "network/stk_host.hpp"
#include "utils/time.hpp"

KartUpdateProtocol::KartUpdateProtocol() : Protocol(PROTOCOL_KART_UPDATE)
{
}   // KartUpdateProtocol

// ----------------------------------------------------------------------------
KartUpdateProtocol::~KartUpdateProtocol()
{
}   // ~KartUpdateProtocol

// ----------------------------------------------------------------------------
void KartUpdateProtocol::setup()
{
    // Allocate arrays to store one position and rotation for each kart
    // (which is the update information from the server to the client).
    m_all_updates.resize(World::getWorld()->getNumKarts());
    for(unsigned int i=0; i<m_all_updates.size(); i++)
    {
        m_all_updates[i].resize(3);
    }

    // This flag keeps track if valid data for an update is in
    // the arrays
    m_was_updated          = false;
    m_previous_update_time = 0;
    m_force_update         = false;
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
#ifdef LOG_UPDATED
    Log::error("update received", "%f %f %f %f %f",
        my_time, next_time,
        m_all_updates[0][0].m_server_time,
        m_all_updates[0][1].m_server_time,
        m_all_updates[0][2].m_server_time
        );
#endif

    while(ns.size() >= 29)
    {
        uint8_t kart_id             = ns.getUInt8();
        KartUpdate ka_new;
        ka_new.m_server_time        = next_time;
        ka_new.m_xyz                = ns.getVec3();
        ka_new.m_quat               = ns.getQuat();
        std::vector<KartUpdate> &ka = m_all_updates[kart_id];

        if(ka[0].m_server_time<0)
        {
            // First update ever received. 
            ka[0] = ka_new; ka[0].m_server_time = ka_new.m_server_time - 0.02f;
            // Avoid division by zero in interpolation code
            ka[1] = ka_new; ka[1].m_server_time = ka_new.m_server_time - 0.01f;
            ka[2] = ka_new; ka[2].m_server_time = ka_new.m_server_time;
        }
        else if(next_time < my_time)
        {
            // client ahead of server :(
            // This should not happen, hopefully it's caused by network
            // delays and will sort itself out shortly. This will lead
            // to extrapolation and shaking, but we can't do much about this
            // in a dumb client.
            if(ka[2].m_server_time < my_time)
            {
                // Save the previous latest update, which is now before local
                // time
                ka[1] = ka[2];
            }
            ka[2] = ka_new;
        }
        else if(next_time > ka[2].m_server_time)
        {
            // update 1 is behind local time, the new one is ahead, so
            // just save the latest update in 2

            // If the current latest is before local time, move it to update 1
            if(ka[2].m_server_time < my_time)
            {
                // Save the previous latest update, which is now before local
                // time
                ka[1] = ka[2];
            }
            else if(ka[1].m_server_time < my_time)
            {
                // if the update 1 time is indeed before local time, move it
                // to update 0; and move update 2 (which atm is ahead of 
                // local time) to update 1. The interpolation will now use
                // upate 0 and 1
                ka[0] = ka[1];
                ka[1] = ka[2];
            }
            ka[2] = ka_new;
        }
        else   // next_time <= ka[2]
        {
            // The received package is older than the latest udpate from the
            // server, so an out-of-order update. If possible use this data
            // as 'previous' update if it is later than the current previous
            // update and still behind the local time
            if(next_time > ka[1].m_server_time && next_time > my_time)
                ka[1] = ka_new;
        }
        
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

    float current_time = float(StkTime::getRealTime());
    // Dumb clients need updates as often as possible.
    // Otherwise update 10 times a second only
    if (NetworkConfig::get()->isServer() )
    {
        //if( current_time > m_previous_update_time + 0.1 || m_force_update )
        if( NetworkConfig::get()->useDumbClient() ||
            current_time > m_previous_update_time + 0.1f)
        {
            m_previous_update_time = current_time;
            sendKartUpdates();
            m_force_update = false;
        }   // if (current_time > time + 0.1)
        return;
    }   // if server

    // Now handle all update events that have been received on a client.
    // There is no lock necessary, since receiving new positions is done in
    // notifyEvent, which is called from the same thread that calls this
    // function.
    float my_time = World::getWorld()->getTime();
    for(unsigned i=0; i<World::getWorld()->getNumKarts(); i++)
    {
        AbstractKart *kart = World::getWorld()->getKart(i);
        std::vector<KartUpdate> &ku = m_all_updates[i];
        KartUpdate prev, next;
        if(my_time >= ku[1].m_server_time)   // inteprolate between 1 and 2
        {
            prev = ku[1];
            next = ku[2];
        }
        else                   // interpolate between 0 and 1
        {
            prev = ku[0];
            next = ku[1];
        }
        // Don't change anything if there was no update
        if(prev.m_server_time == next.m_server_time) continue;
        float dt_server = next.m_server_time - prev.m_server_time;
        float f = (my_time            - prev.m_server_time )
                / (next.m_server_time - prev.m_server_time );
        // A bad hack to prevent extrapolation, which results in
        // very shaky game play
        if(f>1) f=1.0f;
        Vec3 xyz = prev.m_xyz +  (next.m_xyz - prev.m_xyz)*f;
#ifdef LOG_POSITION_AND_TIME
        Log::error("xyz", "%f %f %f %f  y %f %f %f %f f %f events %d enet %d",
            World::getWorld()->getTime(), ku[0].m_server_time,
            ku[1].m_server_time, ku[2].m_server_time,
            kart->getXYZ().getY(),
            prev.m_xyz.getY(),
            xyz.getY(),
            next.m_xyz.getY(),
            f,
            ProtocolManager::getInstance()->getNumEvents(),
            STKHost::get()->getEnetQueueLength()
            );
#endif
        kart->setXYZ(xyz);
        btQuaternion q = prev.m_quat.slerp(next.m_quat, f);
        kart->setRotation(q);

    }   // for i < number of karts

    // Adjust
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

// ----------------------------------------------------------------------------
/** This function forces this protocol to send a kart update next time update
 *  is called. This is used to quickly update clients when  a kart state
 *  changes, without waiting up to 1/10 of a second.
 */
void KartUpdateProtocol::forceUpdateSending()
{
    m_force_update = true;
}   // forceUpdateSending
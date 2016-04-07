#ifndef KART_UPDATE_PROTOCOL_HPP
#define KART_UPDATE_PROTOCOL_HPP

#include "network/protocol.hpp"
#include "utils/cpp2011.hpp"
#include "utils/vec3.hpp"

#include "LinearMath/btQuaternion.h"

#include <vector>
#include "pthread.h"

class AbstractKart;

class KartUpdateProtocol : public Protocol
{
private:

    /** Stores a single update for a kart. */
    class KartUpdate
    {
    public:
        /** Server game time at which this update was sent. */
        float        m_server_time;
        /** Position of the kart at the given time. */
        Vec3         m_xyz;
        /** Rotation of the kart at the given time. */
        btQuaternion m_quat;
        // --------------------------------------------------------------------
        /** Default constructor to initialise the data. Sets everything to 0.*/
        KartUpdate()
        {
            m_server_time = -1;
            m_xyz         = Vec3(0,0,0);
            m_quat        = btQuaternion(0,0,0,1);
        }   // KartUpdate
        // --------------------------------------------------------------------
        /** Sets all values of this update. */
        void set(float time, const Vec3 &xyz, const btQuaternion &quat)
        {
            m_server_time = time;
            m_xyz         = xyz;
            m_quat        = quat;
        }   // set
    };   // struct KartUpdate

    // ========================================================================
    /** The list of all updates n received for a kart, sorted
     *  by time: m_all_updates[kartid][n] > m_all_updates[kartid][n-1].
     *  For each kart, three values are stored:
     *  0:  An update before the local game time.
     *  1:  The smallest time at or later as the local game time.
     *  2:  The latest server update.
     *  Ideally the local game time should be between 1 and 2, but especially
     *  at startup the server might be be ahead of the client.
     *  The client karts will interpolate either between 1 and 2, or 0 and 2.
     */
    std::vector< std::vector<KartUpdate> > m_all_updates;

    /** True if a new update for the kart positions was received. */
    bool m_was_updated;

    /** Time the last kart update was sent. Used to send updates with
     * a fixed frequency. */
    float m_previous_update_time;

    /** When this is set, the server will sent an update to the clients even
     *  if the normal update frequency is not reached. */
    bool m_force_update;

    void sendKartUpdates();

public:
             KartUpdateProtocol();
    virtual ~KartUpdateProtocol();

    void forceUpdateSending();
    virtual bool notifyEvent(Event* event) OVERRIDE;
    virtual void setup() OVERRIDE;
    virtual void update(float dt) OVERRIDE;
    virtual void asynchronousUpdate() OVERRIDE {};

};   // KartUpdateProtocol

#endif // KART_UPDATE_PROTOCOL_HPP

//
//  SuperTuxKart - a fun racing game with go-kart
//  Copyright (C) 2006-2015 SuperTuxKart-Team
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the GNU General Public License
//  as published by the Free Software Foundation; either version 3
//  of the License, or (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.

#include "modes/world.hpp"

#include "achievements/achievement_info.hpp"
#include "audio/music_manager.hpp"
#include "audio/sfx_base.hpp"
#include "audio/sfx_manager.hpp"
#include "config/player_manager.hpp"
#include "challenges/unlock_manager.hpp"
#include "config/user_config.hpp"
#include "graphics/camera.hpp"
#include "graphics/irr_driver.hpp"
#include "graphics/material.hpp"
#include "graphics/material_manager.hpp"
#include "graphics/render_info.hpp"
#include "io/file_manager.hpp"
#include "input/device_manager.hpp"
#include "input/keyboard_device.hpp"
#include "items/projectile_manager.hpp"
#include "karts/controller/battle_ai.hpp"
#include "karts/controller/soccer_ai.hpp"
#include "karts/controller/end_controller.hpp"
#include "karts/controller/local_player_controller.hpp"
#include "karts/controller/skidding_ai.hpp"
#include "karts/controller/spare_tire_ai.hpp"
#include "karts/controller/test_ai.hpp"
#include "karts/controller/network_player_controller.hpp"
#include "karts/kart.hpp"
#include "karts/kart_properties_manager.hpp"
#include "karts/kart_rewinder.hpp"
#include "modes/overworld.hpp"
#include "modes/profile_world.hpp"
#include "modes/soccer_world.hpp"
#include "network/network_config.hpp"
#include "network/rewind_manager.hpp"
#include "physics/btKart.hpp"
#include "physics/physics.hpp"
#include "physics/triangle_mesh.hpp"
#include "race/highscore_manager.hpp"
#include "race/history.hpp"
#include "race/race_manager.hpp"
#include "replay/replay_play.hpp"
#include "replay/replay_recorder.hpp"
#include "scriptengine/script_engine.hpp"
#include "states_screens/dialogs/race_paused_dialog.hpp"
#include "states_screens/race_gui_base.hpp"
#include "states_screens/main_menu_screen.hpp"
#include "states_screens/race_gui.hpp"
#include "states_screens/race_result_gui.hpp"
#include "states_screens/state_manager.hpp"
#include "tracks/track.hpp"
#include "tracks/track_manager.hpp"
#include "utils/constants.hpp"
#include "utils/profiler.hpp"
#include "utils/translation.hpp"
#include "utils/string_utils.hpp"

#include <algorithm>
#include <assert.h>
#include <ctime>
#include <sstream>
#include <stdexcept>


World* World::m_world = NULL;

/** The main world class is used to handle the track and the karts.
 *  The end of the race is detected in two phases: first the (abstract)
 *  function isRaceOver, which must be implemented by all game modes,
 *  must return true. In which case enterRaceOverState is called. At
 *  this time a winning (or losing) animation can be played. The WorldStatus
 *  class will in its enterRaceOverState switch to DELAY_FINISH_PHASE,
 *  but the remaining AI kart will keep on racing during that time.
 *  After a time period specified in stk_config.xml WorldStatus will
 *  switch to FINISH_PHASE and call terminateRace. Now the finishing status
 *  of all karts is set (i.e. in a normal race the arrival time for karts
 *  will be estimated), highscore is updated, and the race result gui
 *  is being displayed.
 *  Rescuing is handled via the three functions:
 *  getNumberOfRescuePositions() - which returns the number of rescue
 *           positions defined.
 *  getRescuePositionIndex(AbstractKart *kart) - which determines the
 *           index of the rescue position to be used for the given kart.
 *  getRescueTransform(unsigned int index) - which returns the transform
 *           (i.e. position and rotation) for the specified rescue
 *           position.
 *  This allows the world class to do some tests to make sure all rescue
 *  positions are valid (when started with --track-debug). It tries to
 *  place all karts on all rescue positions. If there are any problems
 *  (e.g. a rescue position not over terrain (perhaps because it is too
 *  low); or the rescue position is on a texture which will immediately
 *  trigger another rescue), a warning message will be printed.
 */

//-----------------------------------------------------------------------------
/** Constructor. Note that in the constructor it is not possible to call any
 *  functions that use World::getWorld(), since this is only defined
 *  after the constructor. Those functions must be called in the init()
 *  function, which is called immediately after the constructor.
 */
World::World() : WorldStatus()
{

#ifdef DEBUG
    m_magic_number = 0xB01D6543;
#endif

    m_race_gui           = NULL;
    m_saved_race_gui     = NULL;
    m_use_highscores     = true;
    m_schedule_pause     = false;
    m_schedule_unpause   = false;
    m_schedule_exit_race = false;
    m_self_destruct      = false;
    m_schedule_tutorial  = false;
    m_is_network_world   = false;

    m_stop_music_when_dialog_open = true;

    WorldStatus::setClockMode(CLOCK_CHRONO);

}   // World

// ----------------------------------------------------------------------------
/** This function is called after instanciating. The code here can't be moved
 *  to the contructor as child classes must be instanciated, otherwise
 *  polymorphism will fail and the results will be incorrect . Also in init()
 *  functions can be called that use World::getWorld().
 */
void World::init()
{
    m_faster_music_active = false;
    m_fastest_kart        = 0;
    m_eliminated_karts    = 0;
    m_eliminated_players  = 0;
    m_num_players         = 0;
    unsigned int gk       = 0;
    if (race_manager->hasGhostKarts())
        gk = ReplayPlay::get()->getNumGhostKart();

    // Create the race gui before anything else is attached to the scene node
    // (which happens when the track is loaded). This allows the race gui to
    // do any rendering on texture. Note that this function can NOT be called
    // in the World constuctor, since it might be overwritten by a the game
    // mode class, which would not have been constructed at the time that this
    // constructor is called, so the wrong race gui would be created.
    createRaceGUI();

    RewindManager::create();

    // Grab the track file
    Track *track = track_manager->getTrack(race_manager->getTrackName());
    Scripting::ScriptEngine::getInstance<Scripting::ScriptEngine>();
    if(!track)
    {
        std::ostringstream msg;
        msg << "Track '" << race_manager->getTrackName()
            << "' not found.\n";
        throw std::runtime_error(msg.str());
    }

    std::string script_path = track->getTrackFile("scripting.as");
    Scripting::ScriptEngine::getInstance()->loadScript(script_path, true);

    // Create the physics
    Physics::getInstance<Physics>();

    unsigned int num_karts = race_manager->getNumberOfKarts();
    //assert(num_karts > 0);

    // Load the track models - this must be done before the karts so that the
    // karts can be positioned properly on (and not in) the tracks.
    // This also defines the static Track::getCurrentTrack function.
    track->loadTrackModel(race_manager->getReverseTrack());

    if (gk > 0)
    {
        ReplayPlay::get()->load();
        for (unsigned int k = 0; k < gk; k++)
            m_karts.push_back(ReplayPlay::get()->getGhostKart(k));
    }

    // Assign team of AIs for soccer mode before createKart
    SoccerWorld* sw = dynamic_cast<SoccerWorld*>(this);
    if (sw)
        sw->setAITeam();

    for(unsigned int i=0; i<num_karts; i++)
    {
        if (race_manager->getKartType(i) == RaceManager::KT_GHOST) continue;
        std::string kart_ident = history->replayHistory()
                               ? history->getKartIdent(i)
                               : race_manager->getKartIdent(i);
        int local_player_id  = race_manager->getKartLocalPlayerId(i);
        int global_player_id = race_manager->getKartGlobalPlayerId(i);
        AbstractKart* newkart = createKart(kart_ident, i, local_player_id,
                                   global_player_id,
                                   race_manager->getKartType(i),
                                   race_manager->getPlayerDifficulty(i));
        m_karts.push_back(newkart);
        track->adjustForFog(newkart->getNode());

    }  // for i

    // Load other custom models if needed
    loadCustomModels();

#ifndef SERVER_ONLY
    // Now that all models are loaded, apply the overrides
    irr_driver->applyObjectPassShader();
#endif

    // Must be called after all karts are created
    m_race_gui->init();

    powerup_manager->updateWeightsForRace(race_manager->getNumberOfKarts());

    if (UserConfigParams::m_weather_effects)
    {
        Weather::getInstance<Weather>();   // create Weather instance
    }
}   // init

//-----------------------------------------------------------------------------
/** This function is called before a race is started (i.e. either after
 *  calling init() when starting a race for the first time, or after
 *  restarting a race, in which case no init() is called.
 */
void World::reset()
{
    RewindManager::get()->reset();

    // If m_saved_race_gui is set, it means that the restart was done
    // when the race result gui was being shown. In this case restore the
    // race gui (note that the race result gui is cached and so never really
    // destroyed).
    if(m_saved_race_gui)
    {
        m_race_gui       = m_saved_race_gui;
        m_saved_race_gui = NULL;
    }

    m_schedule_pause = false;
    m_schedule_unpause = false;

    WorldStatus::reset();
    m_faster_music_active = false;
    m_eliminated_karts    = 0;
    m_eliminated_players  = 0;
    m_is_network_world = false;

    for ( KartList::iterator i = m_karts.begin(); i != m_karts.end() ; ++i )
    {
        (*i)->reset();
    }

    Camera::resetAllCameras();

    if(race_manager->hasGhostKarts())
        ReplayPlay::get()->reset();

    resetAllKarts();
    // Note: track reset must be called after all karts exist, since check
    // objects need to allocate data structures depending on the number
    // of karts.
    Track::getCurrentTrack()->reset();

    // Reset the race gui.
    m_race_gui->reset();

    // Start music from beginning
    music_manager->stopMusic();

    // Enable SFX again
    SFXManager::get()->resumeAll();

    projectile_manager->cleanup();
    race_manager->reset();
    // Make sure to overwrite the data from the previous race.
    if(!history->replayHistory()) history->initRecording();
    if(race_manager->isRecordingRace())
    {
        loginfo("World", "Start Recording race.");
        ReplayRecorder::get()->init();
    }
    if((NetworkConfig::get()->isServer() && !ProfileWorld::isNoGraphics()) ||
        race_manager->isWatchingReplay())
    {
        // In case that the server is running with gui or watching replay,
        // create a camera and attach it to the first kart.
        Camera::createCamera(World::getWorld()->getKart(0));

    }

    // Reset all data structures that depend on number of karts.
    irr_driver->reset();

    //Reset the Rubber Ball Collect Time to some negative value.
    powerup_manager->setBallCollectTime(-100);
}   // reset

//-----------------------------------------------------------------------------

void World::createRaceGUI()
{
    m_race_gui = new RaceGUI();
}

//-----------------------------------------------------------------------------
/** Creates a kart, having a certain position, starting location, and local
 *  and global player id (if applicable).
 *  \param kart_ident Identifier of the kart to create.
 *  \param index Index of the kart.
 *  \param local_player_id If the kart is a player kart this is the index of
 *         this player on the local machine.
 *  \param global_player_id If the kart is a player kart this is the index of
 *         this player globally (i.e. including network players).
 */
AbstractKart *World::createKart(const std::string &kart_ident, int index,
                                int local_player_id, int global_player_id,
                                RaceManager::KartType kart_type,
                                PerPlayerDifficulty difficulty)
{
    unsigned int gk = 0;
    if (race_manager->hasGhostKarts())
        gk = ReplayPlay::get()->getNumGhostKart();

    int position           = index+1;
    btTransform init_pos   = getStartTransform(index - gk);
    AbstractKart *new_kart;
    if (RewindManager::get()->isEnabled())
        new_kart = new KartRewinder(kart_ident, index, position, init_pos,
                                    difficulty);
    else
        new_kart = new Kart(kart_ident, index, position, init_pos, difficulty);

    new_kart->init(race_manager->getKartType(index));
    Controller *controller = NULL;
    switch(kart_type)
    {
    case RaceManager::KT_PLAYER:
        controller = new LocalPlayerController(new_kart,
                         StateManager::get()->getActivePlayer(local_player_id));
        m_num_players ++;
        break;
    case RaceManager::KT_NETWORK_PLAYER:
        controller = new NetworkPlayerController(new_kart);
        m_num_players++;
        break;
    case RaceManager::KT_AI:
        controller = loadAIController(new_kart);
        break;
    case RaceManager::KT_GHOST:
        break;
    case RaceManager::KT_LEADER:
        break;
    case RaceManager::KT_SPARE_TIRE:
        break;
    }

    new_kart->setController(controller);

    return new_kart;
}   // createKart

//-----------------------------------------------------------------------------
/** Returns the start coordinates for a kart with a given index.
 *  \param index Index of kart ranging from 0 to kart_num-1. */
const btTransform &World::getStartTransform(int index)
{
    return Track::getCurrentTrack()->getStartTransform(index);
}   // getStartTransform

//-----------------------------------------------------------------------------
/** Creates an AI controller for the kart.
 *  \param kart The kart to be controlled by an AI.
 */
Controller* World::loadAIController(AbstractKart *kart)
{
    Controller *controller;
    int turn=0;

    if(race_manager->getMinorMode()==RaceManager::MINOR_MODE_3_STRIKES)
        turn=1;
    else if(race_manager->getMinorMode()==RaceManager::MINOR_MODE_SOCCER)
        turn=2;
    // If different AIs should be used, adjust turn (or switch randomly
    // or dependent on difficulty)
    switch(turn)
    {
        case 0:
            // If requested, start the test ai
            if( (AIBaseController::getTestAI()!=0                       ) && 
                ( (kart->getWorldKartId()+1) % AIBaseController::getTestAI() )==0)
                controller = new TestAI(kart);
            else
                controller = new SkiddingAI(kart);
            break;
        case 1:
            controller = new BattleAI(kart);
            break;
        case 2:
            controller = new SoccerAI(kart);
            break;
        default:
            logwarn("[World]", "Unknown AI, using default.");
            controller = new SkiddingAI(kart);
            break;
    }

    return controller;
}   // loadAIController

//-----------------------------------------------------------------------------
World::~World()
{
    material_manager->unloadAllTextures();
    RewindManager::destroy();

    irr_driver->onUnloadWorld();

    // In case that a race is aborted (e.g. track not found) track is 0.
    if(Track::getCurrentTrack())
        Track::getCurrentTrack()->cleanup();

    // Delete the in-race-gui:
    if(m_saved_race_gui)
    {
        // If there is a save race gui, this means that the result gui is
        // currently being shown. The race result gui is a screen and so
        // is deleted by the state manager. So we only have to delete
        // the actual race gui:
        delete m_saved_race_gui;
    }
    else
    {
        // No race result gui is shown, so m_race_gui is the in-race
        // gui and this must be deleted.
        delete m_race_gui;
    }
    
    Weather::kill();

    for ( unsigned int i = 0 ; i < m_karts.size() ; i++ )
    {
        // Let ReplayPlay destroy the ghost karts
        if (m_karts[i]->isGhostKart()) continue;
        delete m_karts[i];
    }

    if(race_manager->hasGhostKarts() || race_manager->isRecordingRace())
    {
        // Destroy the old replay object, which also stored the ghost
        // karts, and create a new one (which means that in further
        // races the usage of ghosts will still be enabled).
        // It can allow auto recreation of ghost replay file lists
        // when next time visit the ghost replay selection screen.
        ReplayPlay::destroy();
        ReplayPlay::create();
    }
    m_karts.clear();
    if(race_manager->isRecordingRace())
        ReplayRecorder::get()->reset();
    race_manager->setRaceGhostKarts(false);
    race_manager->setRecordRace(false);
    race_manager->setWatchingReplay(false);
    race_manager->setTimeTarget(0.0f);
    race_manager->setSpareTireKartNum(0);

    Camera::removeAllCameras();

    projectile_manager->cleanup();

    // In case that the track is not found, Physics was not instantiated,
    // but kill handles this correctly.
    Physics::kill();

    Scripting::ScriptEngine::kill();

    m_world = NULL;

    irr_driver->getSceneManager()->clear();

#ifdef DEBUG
    m_magic_number = 0xDEADBEEF;
#endif

}   // ~World

//-----------------------------------------------------------------------------
/** Called when 'go' is being displayed for the first time. Here the brakes
 *  of the karts are released.
 */
void World::onGo()
{
    // Reset the brakes now that the prestart
    // phase is over (braking prevents the karts
    // from sliding downhill)
    for(unsigned int i=0; i<m_karts.size(); i++)
    {
        if (m_karts[i]->isGhostKart()) continue;
        m_karts[i]->getVehicle()->setAllBrakes(0);
    }
}   // onGo

//-----------------------------------------------------------------------------
/** Called at the end of a race. Updates highscores, pauses the game, and
 *  informs the unlock manager about the finished race. This function must
 *  be called after all other stats were updated from the different game
 *  modes.
 */
void World::terminateRace()
{
    m_schedule_pause = false;
    m_schedule_unpause = false;

    // Update the estimated finishing time for all karts that haven't
    // finished yet.
    const unsigned int kart_amount = getNumKarts();
    for(unsigned int i = 0; i < kart_amount ; i++)
    {
        if(!m_karts[i]->hasFinishedRace() && !m_karts[i]->isEliminated())
        {
            m_karts[i]->finishedRace(estimateFinishTimeForKart(m_karts[i]));

        }
    }   // i<kart_amount

    // Update highscores, and retrieve the best highscore if relevant
    // to show it in the GUI
    int best_highscore_rank = -1;
    std::string highscore_who = "";
    if (!this->isNetworkWorld())
    {
        updateHighscores(&best_highscore_rank);
    }

    // Check achievements
    PlayerManager::increaseAchievement(AchievementInfo::ACHIEVE_COLUMBUS,
                                       Track::getCurrentTrack()->getIdent(), 1);
    if (raceHasLaps())
    {
        PlayerManager::increaseAchievement(AchievementInfo::ACHIEVE_MARATHONER,
                                           "laps", race_manager->getNumLaps());
    }

    Achievement *achiev = PlayerManager::getCurrentAchievementsStatus()->getAchievement(AchievementInfo::ACHIEVE_GOLD_DRIVER);
    if (achiev)
    {
        std::string mode_name = getIdent(); // Get the race mode name
        int winner_position = 1;
        unsigned int opponents = achiev->getInfo()->getGoalValue("opponents"); // Get the required opponents number
        if (mode_name == IDENT_FTL)
        {
            winner_position = 2;
            opponents++;
        }
        for(unsigned int i = 0; i < kart_amount; i++)
        {
            // Retrieve the current player
            if (m_karts[i]->getController()->canGetAchievements())
            {
                // Check if the player has won
                if (m_karts[i]->getPosition() == winner_position && kart_amount > opponents )
                {
                    // Update the achievement
                    mode_name = StringUtils::toLowerCase(mode_name);
                    if (achiev->getValue("opponents") <= 0)
                        PlayerManager::increaseAchievement(AchievementInfo::ACHIEVE_GOLD_DRIVER,
                                                            "opponents", opponents);
                    PlayerManager::increaseAchievement(AchievementInfo::ACHIEVE_GOLD_DRIVER,
                                                        mode_name, 1);
                }
            }
        } // for i < kart_amount
    } // if (achiev)

    Achievement *win = PlayerManager::getCurrentAchievementsStatus()->getAchievement(AchievementInfo::ACHIEVE_UNSTOPPABLE);
    //if achivement has been unlocked
    if (win->getValue("wins") < 5 )
    {
        for(unsigned int i = 0; i < kart_amount; i++)
        {
            // Retrieve the current player
            if (m_karts[i]->getController()->canGetAchievements())
            {
                // Check if the player has won
                if (m_karts[i]->getPosition() == 1 )
                {
                    // Increase number of consecutive wins
                       PlayerManager::increaseAchievement(AchievementInfo::ACHIEVE_UNSTOPPABLE,
                                                            "wins", 1);
                }
                else
                {
                      //Set number of consecutive wins to 0
                      win->reset();
                }
            }
         }
    }
    PlayerManager::getCurrentPlayer()->raceFinished();

    if (m_race_gui) m_race_gui->clearAllMessages();
    // we can't delete the race gui here, since it is needed in case of
    // a restart: the constructor of it creates some textures which assume
    // that no scene nodes exist. In case of a restart there are scene nodes,
    // so we can't create the race gui again, so we keep it around
    // and save the pointer.
    assert(m_saved_race_gui==NULL);
    m_saved_race_gui = m_race_gui;

    RaceResultGUI* results = RaceResultGUI::getInstance();
    m_race_gui       = results;

    if (best_highscore_rank > 0)
    {
        results->setHighscore(best_highscore_rank);
    }
    else
    {
        results->clearHighscores();
    }

    results->push();
    WorldStatus::terminateRace();
}   // terminateRace

//-----------------------------------------------------------------------------
/** Waits till each kart is resting on the ground
 *
 * Does simulation steps still all karts reach the ground, i.e. are not
 * moving anymore
 */
void World::resetAllKarts()
{
    // Reset the physics 'remaining' time to 0 so that the number
    // of timesteps is reproducible if doing a physics-based history run
    Physics::getInstance()->getPhysicsWorld()->resetLocalTime();

    // If track checking is requested, check all rescue positions if
    // they are high enough.
    if(UserConfigParams::m_track_debug)
    {
        // Loop over all karts, in case that some karts are dfferent
        for(unsigned int kart_id=0; kart_id<(unsigned int)m_karts.size(); kart_id++)
        {
            if (m_karts[kart_id]->isGhostKart()) continue;
            for(unsigned int rescue_pos=0;
                rescue_pos<getNumberOfRescuePositions();
                rescue_pos++)
            {
                btTransform t = getRescueTransform(rescue_pos);
                // This will print out warnings if there is no terrain under
                // the kart, or the kart is being dropped on a reset texture
                moveKartTo(m_karts[kart_id], t);

            }   // rescue_pos<getNumberOfRescuePositions

            // Reset the karts back to the original start position.
            // This call is a bit of an overkill, but setting the correct
            // transforms, positions, motion state is a bit of a hassle.
            m_karts[kart_id]->reset();
        }   // for kart_id<m_karts.size()


    }   // if m_track_debug

    m_schedule_pause = false;
    m_schedule_unpause = false;

    //Project karts onto track from above. This will lower each kart so
    //that at least one of its wheel will be on the surface of the track
    for ( KartList::iterator i=m_karts.begin(); i!=m_karts.end(); i++)
    {
        if ((*i)->isGhostKart()) continue;
        Vec3 xyz = (*i)->getXYZ();
        //start projection from top of kart
        Vec3 up_offset = (*i)->getNormal() * (0.5f * ((*i)->getKartHeight()));
        (*i)->setXYZ(xyz+up_offset);

        bool kart_over_ground = Track::getCurrentTrack()->findGround(*i);

        if (!kart_over_ground)
        {
            logerror("World",
                       "No valid starting position for kart %d on track %s.",
                       (int)(i - m_karts.begin()),
                       Track::getCurrentTrack()->getIdent().c_str());
            if (UserConfigParams::m_artist_debug_mode)
            {
                logwarn("World", "Activating fly mode.");
                (*i)->flyUp();
                continue;
            }
            else
            {
                exit(-1);
            }
        }
    }

    // Do a longer initial simulation, which should be long enough for all
    // karts to be firmly on ground.
    float g = Track::getCurrentTrack()->getGravity();
    for (KartList::iterator i = m_karts.begin(); i != m_karts.end(); i++)
    {
        if ((*i)->isGhostKart()) continue;
        (*i)->getBody()->setGravity((*i)->getMaterial()->hasGravity() ?
            (*i)->getNormal() * -g : Vec3(0, -g, 0));
    }
    for(int i=0; i<60; i++) Physics::getInstance()->update(1.f/60.f);

    for ( KartList::iterator i=m_karts.begin(); i!=m_karts.end(); i++)
    {
        (*i)->kartIsInRestNow();
    }

    // Initialise the cameras, now that the correct kart positions are set
    for(unsigned int i=0; i<Camera::getNumCameras(); i++)
    {
        Camera::getCamera(i)->setInitialTransform();
    }
}   // resetAllKarts

// ----------------------------------------------------------------------------
/** Places a kart that is rescued. It calls getRescuePositionIndex to find
 *  to which rescue position the kart should be moved, then getRescueTransform
 *  to get the position and rotation of this rescue position, and then moves
 *  the kart.
 *  \param kart The kart that is rescued.
 */
void World::moveKartAfterRescue(AbstractKart* kart)
{
    unsigned int index = getRescuePositionIndex(kart);
    btTransform t      = getRescueTransform(index);
    moveKartTo(kart, t);
}  // moveKartAfterRescue

// ----------------------------------------------------------------------------
/** Places the kart at a given position and rotation.
 *  \param kart The kart to be moved.
 *  \param transform
 */
void World::moveKartTo(AbstractKart* kart, const btTransform &transform)
{
    btTransform pos(transform);

    // Move the kart
    Vec3 xyz = pos.getOrigin() +
        pos.getBasis() * Vec3(0, 0.5f*kart->getKartHeight(), 0);
    pos.setOrigin(xyz);
    kart->setXYZ(xyz);
    kart->setRotation(pos.getRotation());

    kart->getBody()->setCenterOfMassTransform(pos);
    // The raycast to determine the terrain underneath the kart is done from
    // the centre point of the 4 wheel positions. After a rescue, the wheel
    // positions need to be updated (otherwise the raycast will be done from
    // the previous position, which might be the position that triggered
    // the rescue in the first place).
    kart->getVehicle()->updateAllWheelPositions();

    // Project kart to surface of track
    // This will set the physics transform
    Track::getCurrentTrack()->findGround(kart);
    CheckManager::get()->resetAfterKartMove(kart);

}   // moveKartTo

// ----------------------------------------------------------------------------
void World::schedulePause(Phase phase)
{
    if (m_schedule_unpause)
    {
        m_schedule_unpause = false;
    }
    else
    {
        m_schedule_pause = true;
        m_scheduled_pause_phase = phase;
    }
}   // schedulePause

// ----------------------------------------------------------------------------
void World::scheduleUnpause()
{
    if (m_schedule_pause)
    {
        m_schedule_pause = false;
    }
    else
    {
        m_schedule_unpause = true;
    }
}   // scheduleUnpause

//-----------------------------------------------------------------------------
/** This is the main interface to update the world. This function calls
 *  update(), and checks then for the end of the race. Note that race over
 *  handling can not necessarily be done in update(), since not all
 *  data structures might have been updated (e.g.LinearWorld must
 *  call World::update() first, to get updated kart positions. If race
 *  over would be handled in World::update, LinearWorld had no opportunity
 *  to update its data structures before the race is finished).
 *  \param dt Time step size.
 */
void World::updateWorld(float dt)
{
#ifdef DEBUG
    assert(m_magic_number == 0xB01D6543);
#endif


    if (m_schedule_pause)
    {
        pause(m_scheduled_pause_phase);
        m_schedule_pause = false;
    }
    else if (m_schedule_unpause)
    {
        unpause();
        m_schedule_unpause = false;
    }

    if (m_self_destruct)
    {
        delete this;
        return;
    }

    // Don't update world if a menu is shown or the race is over.
    if( getPhase() == FINISH_PHASE         ||
        getPhase() == IN_GAME_MENU_PHASE      )
        return;

    if (!history->replayHistory())
    {
        history->updateSaving(dt);   // updating the saved state
    }

    try
    {
        update(dt);
    }
    catch (AbortWorldUpdateException& e)
    {
        (void)e;   // avoid compiler warning
        return;
    }

#ifdef DEBUG
    assert(m_magic_number == 0xB01D6543);
#endif

    if( (!isFinishPhase()) && isRaceOver())
    {
        enterRaceOverState();
    }
    else
    {
        if (m_schedule_exit_race)
        {
            m_schedule_exit_race = false;
            race_manager->exitRace(false);
            race_manager->setAIKartOverride("");

            StateManager::get()->resetAndGoToScreen(MainMenuScreen::getInstance());

            if (m_schedule_tutorial)
            {
                m_schedule_tutorial = false;
                race_manager->setNumPlayers(1);
                race_manager->setMajorMode (RaceManager::MAJOR_MODE_SINGLE);
                race_manager->setMinorMode (RaceManager::MINOR_MODE_TUTORIAL);
                race_manager->setNumKarts( 1 );
                race_manager->setTrack( "tutorial" );
                race_manager->setDifficulty(RaceManager::DIFFICULTY_EASY);
                race_manager->setReverseTrack(false);

                // Use keyboard 0 by default (FIXME: let player choose?)
                InputDevice* device = input_manager->getDeviceManager()->getKeyboard(0);

                // Create player and associate player with keyboard
                StateManager::get()->createActivePlayer(PlayerManager::getCurrentPlayer(),
                                                        device);

                if (!kart_properties_manager->getKart(UserConfigParams::m_default_kart))
                {
                    logwarn("[World]",
                              "Cannot find kart '%s', will revert to default.",
                              UserConfigParams::m_default_kart.c_str());
                    UserConfigParams::m_default_kart.revertToDefaults();
                }
                race_manager->setPlayerKart(0, UserConfigParams::m_default_kart);

                // ASSIGN should make sure that only input from assigned devices
                // is read.
                input_manager->getDeviceManager()->setAssignMode(ASSIGN);
                input_manager->getDeviceManager()
                    ->setSinglePlayer( StateManager::get()->getActivePlayer(0) );

                delete this;

                StateManager::get()->enterGameState();
                race_manager->setupPlayerKartInfo();
                race_manager->startNew(true);
            }
            else
            {
                delete this;

                if (race_manager->raceWasStartedFromOverworld())
                {
                    OverWorld::enterOverWorld();
                }

            }
        }
    }
}   // updateWorld

#define MEASURE_FPS 0

//-----------------------------------------------------------------------------

void World::scheduleTutorial()
{
    m_schedule_exit_race = true;
    m_schedule_tutorial = true;
}   // scheduleTutorial

//-----------------------------------------------------------------------------
/** Updates the physics, all karts, the track, and projectile manager.
 *  \param dt Time step size.
 */
void World::update(float dt)
{
#ifdef DEBUG
    assert(m_magic_number == 0xB01D6543);
#endif

    PROFILER_PUSH_CPU_MARKER("World::update()", 0x00, 0x7F, 0x00);

#if MEASURE_FPS
    static float time = 0.0f;
    time += dt;
    if (time > 5.0f)
    {
        time -= 5.0f;
        printf("%i\n",irr_driver->getVideoDriver()->getFPS());
    }
#endif

    PROFILER_PUSH_CPU_MARKER("World::update (sub-updates)", 0x20, 0x7F, 0x00);
    WorldStatus::update(dt);
    RewindManager::get()->saveStates();
    PROFILER_POP_CPU_MARKER();

    PROFILER_PUSH_CPU_MARKER("World::update (Kart::upate)", 0x40, 0x7F, 0x00);

    // Update all the karts. This in turn will also update the controller,
    // which causes all AI steering commands set. So in the following 
    // physics update the new steering is taken into account.
    const int kart_amount = (int)m_karts.size();
    for (int i = 0 ; i < kart_amount; ++i)
    {
        SpareTireAI* sta =
            dynamic_cast<SpareTireAI*>(m_karts[i]->getController());
        // Update all karts that are not eliminated
        if(!m_karts[i]->isEliminated() || (sta && sta->isMoving()))
            m_karts[i]->update(dt);
    }
    PROFILER_POP_CPU_MARKER();

    PROFILER_PUSH_CPU_MARKER("World::update (camera)", 0x60, 0x7F, 0x00);
    for(unsigned int i=0; i<Camera::getNumCameras(); i++)
    {
        Camera::getCamera(i)->update(dt);
    }
    PROFILER_POP_CPU_MARKER();

    if(race_manager->isRecordingRace()) ReplayRecorder::get()->update(dt);
    Scripting::ScriptEngine *script_engine = Scripting::ScriptEngine::getInstance();
    if (script_engine) script_engine->update(dt);

    if (!history->dontDoPhysics())
    {
        Physics::getInstance()->update(dt);
    }

    PROFILER_PUSH_CPU_MARKER("World::update (weather)", 0x80, 0x7F, 0x00);
    if (UserConfigParams::m_graphical_effects && Weather::getInstance())
    {
        Weather::getInstance()->update(dt);
    }
    PROFILER_POP_CPU_MARKER();

    PROFILER_PUSH_CPU_MARKER("World::update (projectiles)", 0xa0, 0x7F, 0x00);
    projectile_manager->update(dt);
    PROFILER_POP_CPU_MARKER();

    PROFILER_POP_CPU_MARKER();

#ifdef DEBUG
    assert(m_magic_number == 0xB01D6543);
#endif
}   // update

// ----------------------------------------------------------------------------
/** Compute the new time, and set this new time to be used in the rewind
 *  manager.
 *  \param dt Time step size.
 */
void World::updateTime(const float dt)
{
    WorldStatus::updateTime(dt);
    RewindManager::get()->setCurrentTime(getTime(), dt);
}   // updateTime

// ----------------------------------------------------------------------------
/** Only updates the track. The order in which the various parts of STK are
 *  updated is quite important (i.e. the track can't be updated as part of
 *  the standard update call):
 *  the track must be updated after updating the karts (otherwise the
 *  checklines would be using the previous kart positions to determine
 *  new laps, but linear world which determines distance along track would
 *  be using the new kart positions --> the lap counting line will be
 *  triggered one frame too late, potentially causing strange behaviour of
 *  the icons.
 *  Similarly linear world must update the position of all karts after all
 *  karts have been updated (i.e. World::update() must be called before
 *  updating the position of the karts). The check manager (which is called
 *  from Track::update()) needs the updated distance along track, so track
 *  update has to be called after updating the race position in linear world.
 *  That's why there is a separate call for trackUpdate here.
 */
void World::updateTrack(float dt)
{
    Track::getCurrentTrack()->update(dt);
}   // update Track
// ----------------------------------------------------------------------------

Highscores* World::getHighscores() const
{
    if(!m_use_highscores) return NULL;

    const Highscores::HighscoreType type = "HST_" + getIdent();

    Highscores * highscores =
        highscore_manager->getHighscores(type,
                                         getNumKarts(),
                                         race_manager->getDifficulty(),
                                         race_manager->getTrackName(),
                                         race_manager->getNumLaps(),
                                         race_manager->getReverseTrack());

    return highscores;
}   // getHighscores

// ----------------------------------------------------------------------------
/** Called at the end of a race. Checks if the current times are worth a new
 *  score, if so it notifies the HighscoreManager so the new score is added
 *  and saved.
 */
void World::updateHighscores(int* best_highscore_rank)
{
    *best_highscore_rank = -1;

    if(!m_use_highscores) return;

    // Add times to highscore list. First compute the order of karts,
    // so that the timing of the fastest kart is added first (otherwise
    // someone might get into the highscore list, only to be kicked out
    // again by a faster kart in the same race), which might be confusing
    // if we ever decide to display a message (e.g. during a race)
    unsigned int *index = new unsigned int[m_karts.size()];

    const unsigned int kart_amount = (unsigned int) m_karts.size();
    for (unsigned int i=0; i<kart_amount; i++ )
    {
        index[i] = 999; // first reset the contents of the array
    }
    for (unsigned int i=0; i<kart_amount; i++ )
    {
        const int pos = m_karts[i]->getPosition()-1;
        if(pos < 0 || pos >= (int)kart_amount) continue; // wrong position
        index[pos] = i;
    }

    for (unsigned int pos=0; pos<kart_amount; pos++)
    {
        if(index[pos] == 999)
        {
            // no kart claimed to be in this position, most likely means
            // the kart location data is wrong

#ifdef DEBUG
            logerror("[World]", "Incorrect kart positions:");
            for (unsigned int i=0; i<m_karts.size(); i++ )
            {
                logerror("[World]", "i=%d position %d.",i,
                           m_karts[i]->getPosition());
            }
#endif
            continue;
        }

        // Only record times for local player karts and only if
        // they finished the race
        if(!m_karts[index[pos]]->getController()->isLocalPlayerController())
            continue;
        if (!m_karts[index[pos]]->hasFinishedRace()) continue;

        assert(index[pos] < m_karts.size());
        Kart *k = (Kart*)m_karts[index[pos]];

        Highscores* highscores = getHighscores();

        int highscore_rank = 0;
        // The player is a local player, so there is a name:
        highscore_rank = highscores->addData(k->getIdent(),
                                             k->getController()->getName(),
                                             k->getFinishTime()    );

        if (highscore_rank > 0)
        {
            if (*best_highscore_rank == -1 ||
                highscore_rank < *best_highscore_rank)
            {
                *best_highscore_rank = highscore_rank;
            }

            highscore_manager->saveHighscores();
        }
    } // next position
    delete []index;

}   // updateHighscores

//-----------------------------------------------------------------------------
/** Returns the n-th player kart. Note that this function is O(N), not O(1),
 *  so it shouldn't be called inside of loops.
 *  \param n Index of player kart to return.
 */
AbstractKart *World::getPlayerKart(unsigned int n) const
{
    unsigned int count=-1;

    for(unsigned int i=0; i<m_karts.size(); i++)
        if(m_karts[i]->getController()->isPlayerController())
        {
            count++;
            if(count==n) return m_karts[i];
        }
    return NULL;
}   // getPlayerKart

//-----------------------------------------------------------------------------
/** Returns the nth local player kart, i.e. a kart that has a camera.
 *  Note that in profile mode this means a non player kart could be returned
 *  (since an AI kart will have the camera).
 *  \param n Index of player kart to return.
 */
AbstractKart *World::getLocalPlayerKart(unsigned int n) const
{
    if(n>=Camera::getNumCameras()) return NULL;
    return Camera::getCamera(n)->getKart();
}   // getLocalPlayerKart

//-----------------------------------------------------------------------------
/** Remove (eliminate) a kart from the race */
void World::eliminateKart(int kart_id, bool notify_of_elimination)
{
    AbstractKart *kart = m_karts[kart_id];
    if (kart->isGhostKart()) return;

    // Display a message about the eliminated kart in the race guia
    if (notify_of_elimination)
    {
        for(unsigned int i=0; i<Camera::getNumCameras(); i++)
        {
            Camera *camera = Camera::getCamera(i);
            if(camera->getKart()==kart)
                m_race_gui->addMessage(_("You have been eliminated!"), kart,
                                       2.0f);
            else
                m_race_gui->addMessage(_("'%s' has been eliminated.",
                                       kart->getName()),
                                       camera->getKart(),
                                       2.0f);
        }  // for i < number of cameras
    }   // if notify_of_elimination

    if(kart->getController()->isLocalPlayerController())
    {
        for(unsigned int i=0; i<Camera::getNumCameras(); i++)
        {
            // Change the camera so that it will be attached to the leader
            // and facing backwards.
            Camera *camera = Camera::getCamera(i);
            if(camera->getKart()==kart)
                camera->setMode(Camera::CM_LEADER_MODE);
        }
        m_eliminated_players++;
    }

    // The kart can't be really removed from the m_kart array, since otherwise
    // a race can't be restarted. So it's only marked to be eliminated (and
    // ignored in all loops). Important:world->getCurrentNumKarts() returns
    // the number of karts still racing. This value can not be used for loops
    // over all karts, use race_manager->getNumKarts() instead!
    kart->eliminate();
    m_eliminated_karts++;

}   // eliminateKart

//-----------------------------------------------------------------------------
/** Called to determine the default collectibles to give each player at the
 *  start for this kind of race. Both parameters are of 'out' type.
 *  \param collectible_type The type of collectible each kart.
 *  \param amount The number of collectibles.
 */
void World::getDefaultCollectibles(int *collectible_type, int *amount )
{
    *collectible_type = PowerupManager::POWERUP_NOTHING;
    *amount = 0;
}   // getDefaultCollectibles

//-----------------------------------------------------------------------------
/** Pauses the music (and then pauses WorldStatus).
 */
void World::pause(Phase phase)
{
    if (m_stop_music_when_dialog_open)
        music_manager->pauseMusic();
    SFXManager::get()->pauseAll();

    WorldStatus::pause(phase);
}   // pause

//-----------------------------------------------------------------------------
void World::unpause()
{
    if (m_stop_music_when_dialog_open)
        music_manager->resumeMusic();
    SFXManager::get()->resumeAll();

    WorldStatus::unpause();

    for(unsigned int i=0; i<m_karts.size(); i++)
    {
        // Note that we can not test for isPlayerController here, since
        // an EndController will also return 'isPlayerController' if the
        // kart belonged to a player.
        LocalPlayerController *pc =
            dynamic_cast<LocalPlayerController*>(m_karts[i]->getController());
        if(pc)
            pc->resetInputState();
    }
}   // pause

//-----------------------------------------------------------------------------
/** Call when the world needs to be deleted but you can't do it immediately
 * because you are e.g. within World::update()
 */
void World::delayedSelfDestruct()
{
    m_self_destruct = true;
}   // delayedSelfDestruct

//-----------------------------------------------------------------------------
void World::escapePressed()
{
    new RacePausedDialog(0.8f, 0.6f);
}   // escapePressed

// ----------------------------------------------------------------------------
/** Returns the start transform with the give index.
 *  \param rescue_pos Index of the start position to be returned.
 *  \returns The transform of the corresponding start position.
 */
btTransform World::getRescueTransform(unsigned int rescue_pos) const
{
    return Track::getCurrentTrack()->getStartTransform(rescue_pos);
}   // getRescueTransform

//-----------------------------------------------------------------------------
/** Uses the start position as rescue positions, override if necessary
 */
unsigned int World::getNumberOfRescuePositions() const
{
    return Track::getCurrentTrack()->getNumberOfStartPositions();
}   // getNumberOfRescuePositions

/* EOF */

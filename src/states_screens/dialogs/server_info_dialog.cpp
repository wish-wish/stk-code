//  SuperTuxKart - a fun racing game with go-kart
//  Copyright (C) 2013-2015 Glenn De Jonghe
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

#include "states_screens/dialogs/server_info_dialog.hpp"

#include "audio/sfx_manager.hpp"
#include "guiengine/engine.hpp"
#include "network/protocol_manager.hpp"
#include "network/protocols/request_connection.hpp"
#include "network/servers_manager.hpp"
#include "network/stk_host.hpp"
#include "states_screens/dialogs/registration_dialog.hpp"
#include "states_screens/networking_lobby.hpp"
#include "states_screens/state_manager.hpp"
#include "utils/string_utils.hpp"
#include "utils/translation.hpp"

#include <IGUIEnvironment.h>

using namespace GUIEngine;
using namespace irr;
using namespace irr::gui;
using namespace Online;

// -----------------------------------------------------------------------------
/** Dialog constructor. 
 *  \param server_id ID of the server of which to display the info.
 *  \param host_id ID of the host.
 *  \param from_server_creation: true if the dialog shows the data of this
 *         server (i.e. while it is being created).
 */
ServerInfoDialog::ServerInfoDialog(uint32_t server_id, uint32_t host_id,
                                   bool from_server_creation)
                : ModalDialog(0.8f,0.8f), m_server_id(server_id)
                , m_host_id(host_id)
{
    loginfo("ServerInfoDialog", "Server id is %d, Host id is %d",
               server_id, host_id);
    m_self_destroy = false;
    m_enter_lobby = false;
    m_from_server_creation = from_server_creation;

    loadFromFile("online/server_info_dialog.stkgui");

    GUIEngine::LabelWidget *name = getWidget<LabelWidget>("server_name");
    assert(name);
    const Server * server = ServersManager::get()->getServerByID(m_server_id);
    name->setText(server->getName(),false);

    core::stringw difficulty = race_manager->getDifficultyName(server->getDifficulty());
    GUIEngine::LabelWidget *lbldifficulty = getWidget<LabelWidget>("server_difficulty");
    lbldifficulty->setText(difficulty, false);

    core::stringw mode = RaceManager::getNameOf(server->getRaceMinorMode());
    GUIEngine::LabelWidget *gamemode = getWidget<LabelWidget>("server_game_mode");
    gamemode->setText(mode, false);

    m_info_widget = getWidget<LabelWidget>("info");
    assert(m_info_widget != NULL);
    if (m_from_server_creation)
        m_info_widget->setText(_("Server successfully created. You can now join it."), true);
    m_options_widget = getWidget<RibbonWidget>("options");
    assert(m_options_widget != NULL);
    m_join_widget = getWidget<IconButtonWidget>("join");
    assert(m_join_widget != NULL);
    m_cancel_widget = getWidget<IconButtonWidget>("cancel");
    assert(m_cancel_widget != NULL);
    m_options_widget->setFocusForPlayer(PLAYER_ID_GAME_MASTER);

}   // ServerInfoDialog

// -----------------------------------------------------------------------------

ServerInfoDialog::~ServerInfoDialog()
{
}   // ~ServerInfoDialog

// -----------------------------------------------------------------------------
void ServerInfoDialog::requestJoin()
{
    ServersManager::get()->setJoinedServer(m_server_id);

    STKHost::create();
    ModalDialog::dismiss();
    NetworkingLobby::getInstance()->push();
}   // requestJoin

// -----------------------------------------------------------------------------
GUIEngine::EventPropagation
                 ServerInfoDialog::processEvent(const std::string& eventSource)
{
    if (eventSource == m_options_widget->m_properties[PROP_ID])
    {
        const std::string& selection =
                 m_options_widget->getSelectionIDString(PLAYER_ID_GAME_MASTER);
        if (selection == m_cancel_widget->m_properties[PROP_ID])
        {
            m_self_destroy = true;
            return GUIEngine::EVENT_BLOCK;
        }
        else if(selection == m_join_widget->m_properties[PROP_ID])
        {
            requestJoin();
            return GUIEngine::EVENT_BLOCK;
        }
    }
    return GUIEngine::EVENT_LET;
}   // processEvent

// -----------------------------------------------------------------------------
/** When the player pressed enter, select 'join' as default.
 */
void ServerInfoDialog::onEnterPressedInternal()
{
    // If enter was pressed while none of the buttons was focused interpret
    // as join event
    const int playerID = PLAYER_ID_GAME_MASTER;
    if (GUIEngine::isFocusedForPlayer(m_options_widget, playerID))
        return;
    requestJoin();
}   // onEnterPressedInternal

// -----------------------------------------------------------------------------

bool ServerInfoDialog::onEscapePressed()
{
    if (m_cancel_widget->isActivated())
        m_self_destroy = true;
    return false;
}   // onEscapePressed

// -----------------------------------------------------------------------------
void ServerInfoDialog::onUpdate(float dt)
{
    //If we want to open the registration dialog, we need to close this one first
    if (m_enter_lobby) m_self_destroy = true;

    // It's unsafe to delete from inside the event handler so we do it here
    if (m_self_destroy)
    {
        ModalDialog::dismiss();
        if (m_from_server_creation)
            StateManager::get()->popMenu();
        else if (m_enter_lobby)
            NetworkingLobby::getInstance()->push();
        return;
    }
}   // onUpdate

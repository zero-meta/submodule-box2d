/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef B2_CONTACT_MANAGER_H
#define B2_CONTACT_MANAGER_H

#include "Box2D/Collision/b2BroadPhase.h"
#include "Box2D/Dynamics/b2Fixture.h"
#include "Box2D/Dynamics/Contacts/b2Contact.h"

class b2ContactFilter;
class b2ContactListener;
class b2BlockAllocator;

// Delegate of b2World.
class B2_API b2ContactManager
{
public:
	b2ContactManager();
	~b2ContactManager();

	/// Callback to process a new pair (fixtureA, fixtureB).
	b2Contact* QueryCallback(b2Fixture* fixtureA, b2Fixture* fixtureB);

	// Perform broad-phase collision detection, process the pairs into new contacts and remove dead contacts.
	void FindNewContacts();

	/// Remove contacts that have not persisted, ie currently are not marked with the b2Contact::e_persistFlag.
	void RemoveDeadContacts();

	/// Destroy a contact and remove it from the global contact list.
	/// @warning currently does not removes the contact from the body lists.
	// TODO fix removing from body lists & ensure that the persist flag is not accessed from removed contacts
	void Destroy(b2Contact* c);

	/// Updates the contacts and performs narrow-phase collision detection. May remove some contacts.
	void Collide();

	/// The first contact in this contact manager. Used to iterate the contacts.
	b2Contact* Start() const;

	/// An invalid marker contact that marks the end of the contact list. Used to iterate the contacts.
	b2Contact* End() const;

	/// @return the number of contacts currently in this contact manager.
	int32 GetCount() const;

	b2BroadPhase m_broadPhase;
	int32 m_contactCount;
	b2ContactFilter* m_contactFilter;
	b2ContactListener* m_contactListener;
	b2BlockAllocator* m_allocator;

private:
	b2Contact* m_contactList;
};

inline b2Contact* b2ContactManager::Start() const {
	return m_contactList->m_next;
}

inline b2Contact* b2ContactManager::End() const {
	return m_contactList;
}

inline int32 b2ContactManager::GetCount() const {
	return m_contactCount;
}

#endif

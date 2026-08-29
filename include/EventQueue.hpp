#ifndef EVENT_QUEUE_HPP
#define EVEN

#include <queue>

namespace VexLib{

class Event {
	bool (* func)(void) = nullptr;
public:
	// constructor that can convert a lambda function to an VexLib::Event
	Event(bool(* action)(void));
	Event() = default;
	// returns true if the event got cleared, false otherwise to repeat the action
	virtual bool run();
	
};

class EventQueue{
	std::queue<Event*> events;
public:
	EventQueue() = default;

	/*
	 * @brief adds an event to the list
	 * */
	EventQueue addEvent(Event*);
	
	/* @brief check if the queue is empty
	 * */
	bool isClear();

	/* @brief clear all actions from the queue
	 * */
	void clear();
	

	void update();
};

} // namespace VexLib

#endif

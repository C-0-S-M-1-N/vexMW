#include "EventQueue.hpp"

using VexLib::Event, VexLib::EventQueue;

Event::Event(bool (* f)(void)){ func = f; }
bool Event::run(){ if(func != nullptr) return func(); return true;}


EventQueue EventQueue::addEvent(Event* e){
	events.push(std::move(e));
	return *this;
}

bool EventQueue::isClear(){ return events.empty(); }
void EventQueue::clear(){ while(!events.empty()) events.pop(); }

void EventQueue::update(){
	if(isClear()) return;

	if(events.front()->run()) events.pop();
}



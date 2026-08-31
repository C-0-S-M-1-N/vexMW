#include "EventQueue.hpp"
#include <functional>

using VexLib::Event;
using VexLib::EventQueue;

Event::Event(std::function<bool(void)> f){ func = std::move(f); }
bool Event::run(){ if(func != nullptr) return func(); return true;}


EventQueue EventQueue::addEvent(Event e){
	events.push(e);
	return *this;
}

bool EventQueue::isClear(){ return events.empty(); }
void EventQueue::clear(){ while(!events.empty()) events.pop(); }

void EventQueue::update(){
	if(isClear()) return;

	if(events.front().run()) events.pop();
}



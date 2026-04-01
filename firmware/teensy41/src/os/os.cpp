#include "os.h"
#include "console.h"
#include "utils/interpreter/interpreter.h"

OS OS::m_instance;

OS& os = OS::instance();

FLASHMEM void OS::init(){
    cycle_manager.instance().start();
}

FLASHMEM void OS::run(){
    RUN_EVERY(
    switch(m_state){
        case BOOT:
            boot_routine();
            break;
        case MANUAL:
            manual_routine();
            break;
        case MANUAL_PROGRAM:
            manual_program_routine();
            break;
        case AUTO_PROGRAM:
            auto_program_routine();
            break;
        case AUTO:
            auto_routine();
            break;
        case STOPPED:
            stop_routine();
            break;
        default: 
        break;
    },2)
}

FLASHMEM void OS::start(){
    m_state = AUTO_PROGRAM;
}

FLASHMEM void OS::reboot(){
    CPU_RESTART
}

FLASHMEM void OS::stop(){
    m_state = STOPPED;
}

FLASHMEM void OS::boot_routine(){
    executeRoutine(m_bootRoutine);
    m_state = MANUAL_PROGRAM;
    //Console::println(m_state);
}

FLASHMEM void OS::manual_routine(){
    
    updateServices();
    executeRoutine(m_manualRoutine);

    if(currentJob() != nullptr){
        if(currentJob()->isCompleted() || currentJob()->isCanceled()) killCurrentJob();
        else currentJob()->exec();
        //else if(currentJob()->isIdle()) currentJob()->start();
    }
}

FLASHMEM void OS::auto_routine(){
    updateServices();
    executeRoutine(m_autoRoutine);

    if(currentJob() != nullptr){
        if(currentJob()->isCompleted() || currentJob()->isCanceled()) killCurrentJob();
        else currentJob()->exec();
        //else if(currentJob()->isIdle()) currentJob()->start();
    }
}

FLASHMEM void OS::auto_program_routine(){
    m_state = AUTO;
    executeRoutine(m_auto_programRoutine);
    auto_routine();
    m_state = STOPPED;
}

FLASHMEM void OS::manual_program_routine(){
    m_state = MANUAL;
    updateServices();
    executeRoutine(m_manual_programRoutine);
    manual_routine();
    if(m_state == MANUAL) m_state = MANUAL_PROGRAM;
}

FLASHMEM void OS::stop_routine(){
    executeRoutine(m_stopRoutine);
}

FLASHMEM void OS::setRountine(SystemState state, routine_ptr func_ptr){
    if(func_ptr == nullptr) return;
    switch(state){
        case BOOT:
            m_bootRoutine = func_ptr;
            break;
        case MANUAL:
            m_manualRoutine = func_ptr;
            break;
        case AUTO:
            m_autoRoutine = func_ptr;
            break;
        case MANUAL_PROGRAM:
            m_manual_programRoutine = func_ptr;
        break;
        case AUTO_PROGRAM:
            m_auto_programRoutine = func_ptr;
            break;
        case STOPPED:
            m_stopRoutine = func_ptr;
            break;
        default:
        break;
    }
}

FLASHMEM void OS::setState(SystemState state){
    m_state = state;
}

FLASHMEM void OS::attachService(Service* service){
    if(service != nullptr){
        m_services[service->id()] = service;
        service->attach();
        Console::trace() << "Attached service: " << service->id() << Console::endl;
        service->enable();
    }
}

FLASHMEM bool OS::hasService(ServiceID s) const{
    return m_services.find(s) != m_services.end();
}

FLASHMEM bool OS::statusService(ServiceID serviceID) const{
    if(hasService(serviceID))
        return m_services.at(serviceID)->enabled();
    else
        return false;
}

FLASHMEM bool OS::debug(ServiceID s){
    if(hasService(s))
        return m_services[s]->debug();
    else
        return false;
}

FLASHMEM void OS::toggleDebug(ServiceID s){
    if(hasService(s))
        m_services[s]->toggleDebug();
}

FLASHMEM void OS::wait(unsigned long time) {
    m_timer.setDuration(time);
    m_timer.start();
    // Move the unique_ptr into execute(), which assumes ownership
    execute(m_timer, false);  // Or `runasync` if applicable
}

FLASHMEM void OS::wait(Job& obj, bool isasync) {
    execute(obj, isasync);  // Or `runasync` if applicable
}


FLASHMEM void OS::execute(Job& obj, bool runasync){
    addJob(&obj);
    if(!runasync) while(obj.isPending()) run();
}

FLASHMEM void OS::execute(String& rawcmd){
        Interpreter in;
        in.processScript(rawcmd, script);

        if (script.isValid()) {
            Console::println("Starting program");
            script.start();
            execute(script, false);
        } else {
            Console::println("Invalid program : Unknown error");
        }
}

FLASHMEM void OS::flush(){
    while(isBusy()) run();
}

FLASHMEM bool OS::isBusy() {
    return m_jobs.size() > 0;
}

FLASHMEM void OS::clearProgram(){
    script.clear();
}

Program &OS::program(){
    return script;
};

Job* OS::currentJob(){
    if(m_jobs.size() == 0) return nullptr;
    else return m_jobs.top();
}
FLASHMEM void OS::addJob(Job* job){
    m_jobs.push(job);
}
FLASHMEM void OS::killCurrentJob(){
    if(m_jobs.size() == 0) return;
    m_jobs.pop();
}


FLASHMEM void OS::updateServices(){
    for(const auto& service : m_services) {
        if(service.second->enabled() && !service.second->threaded()) {
            service.second->run();
        }
    }
}

FLASHMEM void OS::enable(ServiceID id){
    if(hasService(id)) m_services[id]->enable();
}

FLASHMEM void OS::disable(ServiceID id){
    if(hasService(id)) m_services[id]->disable();
}

FLASHMEM void OS::executeRoutine(routine_ptr routine){
    if(routine != nullptr){
        routine();
    }else{
        Console::error("OS") << "Routine is nullptr" << Console::endl;
    }
}


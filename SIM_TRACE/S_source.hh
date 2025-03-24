
/**
PURPOSE:
    (Contains sim ojbects)
*/

#ifndef __S_SOURCE_HH__
#define __S_SOURCE_HH__

#include <new>
#include <iostream>
#include <stdlib.h>
#include <cstddef>

#include "trick/attributes.h"
#include "trick/populate_attribute_maps.hh"
#include "trick/ClassSizeCheck.hh"

#include "trick/SimObject.hh"
#include "trick/JobData.hh"
#include "trick/UnitsMap.hh"

#define ip_alloc calloc
#include <sstream>
#include <cstdarg>

#include "trick/AttributesMap.hh"
#include "trick/CheckPointRestart.hh"
#include "trick/CommandLineArguments.hh"
#include "trick/DataRecordDispatcher.hh"
#include "trick/DebugPause.hh"
#include "trick/EchoJobs.hh"
#include "trick/Environment.hh"
#include "trick/EventManager.hh"
#include "trick/EventManager_c_intf.hh"
#include "trick/EventProcessor.hh"
#include "trick/Executive.hh"
#include "trick/ExternalApplicationManager.hh"
#include "trick/FrameLog.hh"
#include "trick/GetTimeOfDayClock.hh"
#include "trick/IPPython.hh"
#include "trick/IPPythonEvent.hh"
#include "trick/ITimer.hh"
#include "trick/IntegLoopManager.hh"
#include "trick/IntegLoopScheduler.hh"
#include "trick/IntegLoopSimObject.hh"
#include "trick/Integrator.hh"
#include "trick/JITEvent.hh"
#include "trick/JITInputFile.hh"
#include "trick/JSONVariableServer.hh"
#include "trick/MSSharedMem.hh"
#include "trick/MSSocket.hh"
#include "trick/MTV.hh"
#include "trick/MalfunctionsTrickView.hh"
#include "trick/Master.hh"
#include "trick/MemoryManager.hh"
#include "trick/MessageCout.hh"
#include "trick/MessageCustomFile.hh"
#include "trick/MessageCustomManager.hh"
#include "trick/MessageFile.hh"
#include "trick/MessageHSFile.hh"
#include "trick/MessageLCout.hh"
#include "trick/MessagePublisher.hh"
#include "trick/MessageSubscriber.hh"
#include "trick/MessageTCDevice.hh"
#include "trick/MessageThreadedCout.hh"
#include "trick/MonteCarlo.hh"
#include "trick/MonteMonitor.hh"
#include "trick/PlaybackFile.hh"
#include "trick/RealtimeSync.hh"
#include "trick/RtiExec.hh"
#include "trick/RtiStager.hh"
#include "trick/Sie.hh"
#include "trick/SimControlPanel.hh"
#include "trick/SimObject.hh"
#include "trick/SimTime.hh"
#include "trick/Slave.hh"
#include "trick/StripChart.hh"
#include "trick/TrickView.hh"
#include "trick/UdUnits.hh"
#include "trick/UnitTest.hh"
#include "trick/VariableServer.hh"
#include "trick/Zeroconf.hh"
#include "trick/collect_proto.hh"
#include "trick/exec_proto.hh"
#include "trick/jit_input_file_proto.hh"

#ifdef __cplusplus
extern "C" {
#endif

#include "trick/clock_proto.h"
#include "trick/data_record_proto.h"
#include "trick/env_proto.h"
#include "trick/exec_proto.h"
#include "trick/memorymanager_c_intf.h"
#include "trick/montecarlo_c_intf.h"
#include "trick/trick_tests.h"
#include "ball/include/ball_numeric.h"

#ifdef __cplusplus
}
#endif





class SimEnvironment : public Trick::Environment {
    public:
       SimEnvironment() ;
} ;


void exec_collect_init( void ) ;

class SysSimObject : public Trick::SimObject {

    public:
        Trick::Executive sched ;

        SysSimObject() {
            Trick::JobData * job __attribute__((unused)) ;
            job = this->add_job(0, 0, "default_data", NULL, 1, "sched.get_freeze_job", "", 60000) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 1, "initialization", NULL, 1, "exec_collect_init", "", 65534) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 2, "initialization", NULL, 1, "sched.write_s_run_summary", "", 65534) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 3, "initialization", NULL, 1, "sched.check_all_jobs_handled", "", 65535) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 4, "initialization", NULL, 1, "sched.check_all_job_cycle_times", "", 65535) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 5, "initialization", NULL, 1, "sched.create_threads", "", 65535) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 6, "initialization", NULL, 1, "sched.write_s_job_execution", "", 65535) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 7, "initialization", NULL, 1, "sched.async_freeze_to_exec_command", "", 65535) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 8, "checkpoint", NULL, 1, "sched.checkpoint", "", 0) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 9, "post_checkpoint", NULL, 1, "sched.post_checkpoint", "", 0) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 10, "restart", NULL, 1, "exec_collect_init", "", 65534) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 11, "system_moding", NULL, 1, "sched.sched_freeze_to_exec_command", "", 60000) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 12, "end_of_frame", NULL, 1, "sched.sched_freeze_to_exec_command", "", 60000) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 13, "end_of_frame", NULL, 1, "sched.async_freeze_to_exec_command", "", 60000) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 14, "freeze_init", NULL, 1, "sched.init_freeze_scheduled", "", 60000) ;
            job = this->add_job(0, 15, "system_advance_sim_time", NULL, 1, "sched.advance_sim_time", "", 60000) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 16, "system_thread_sync", NULL, 1, "sched.thread_sync", "", 60000) ;
            job->add_tag("TRK") ;
        }

    public:
        virtual int call_function( Trick::JobData * curr_job ) ;
        virtual double call_function_double( Trick::JobData * curr_job ) ;


    private:
        // This object is not copyable

        void operator =(const SysSimObject &) {};
} ;

class MonteCarloSimObject : public Trick::SimObject {
    public:
        Trick::MonteCarlo mc ;

        MonteCarloSimObject() {
            Trick::JobData * job __attribute__((unused)) ;
exec_register_scheduler(&mc) ;
            job = this->add_job(0, 0, "default_data", NULL, 1, "mc.process_sim_args", "", 0) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 1, "initialization", NULL, 1, "mc.execute_monte", "", 1) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 2, "shutdown", NULL, 1, "mc.shutdown", "", 60000) ;
            job->add_tag("TRK") ;
        }

    public:
        virtual int call_function( Trick::JobData * curr_job ) ;
        virtual double call_function_double( Trick::JobData * curr_job ) ;

} ;

class MemoryManagerSimObject : public Trick::SimObject {

    public:
        Trick::MemoryManager mm ;

        MemoryManagerSimObject() {
            Trick::JobData * job __attribute__((unused)) ;
}

    public:
        virtual int call_function( Trick::JobData * curr_job ) ;
        virtual double call_function_double( Trick::JobData * curr_job ) ;

} ;

class CheckPointRestartSimObject : public Trick::SimObject {

    public:
        Trick::CheckPointRestart cpr ;

        CheckPointRestartSimObject() {
            Trick::JobData * job __attribute__((unused)) ;
exec_register_scheduler(&cpr) ;
            job = this->add_job(0, 0, "default_data", NULL, 1, "cpr.find_write_checkpoint_jobs", "", 0) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 1, "initialization", NULL, 1, "cpr.write_pre_init_checkpoint", "", 1) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 2, "initialization", NULL, 1, "cpr.write_post_init_checkpoint", "", 65535) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 3, "system_checkpoint", NULL, 1, "cpr.write_checkpoint", "", 0) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 4, "system_checkpoint", NULL, 1, "cpr.safestore_checkpoint", "", 0) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 5, "shutdown", NULL, 1, "cpr.write_end_checkpoint", "", 0) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 6, "freeze", NULL, 1, "cpr.load_checkpoint_job", "", 0) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 7, "end_of_frame", NULL, 1, "cpr.load_checkpoint_job", "", 0) ;
            job->add_tag("TRK") ;
        }

    public:
        virtual int call_function( Trick::JobData * curr_job ) ;
        virtual double call_function_double( Trick::JobData * curr_job ) ;

} ;

class SieSimObject : public Trick::SimObject {
    public:

        Trick::Sie sie ;

        SieSimObject() {
            Trick::JobData * job __attribute__((unused)) ;
            job = this->add_job(0, 0, "default_data", NULL, 1, "sie.process_sim_args", "", 0) ;
            job->add_tag("TRK") ;
            
            
        }

    public:
        virtual int call_function( Trick::JobData * curr_job ) ;
        virtual double call_function_double( Trick::JobData * curr_job ) ;

} ;

class CommandLineArgumentsSimObject : public Trick::SimObject {
    public:
        Trick::CommandLineArguments cmd_args;

        CommandLineArgumentsSimObject() {
            Trick::JobData * job __attribute__((unused)) ;
}

    public:
        virtual int call_function( Trick::JobData * curr_job ) ;
        virtual double call_function_double( Trick::JobData * curr_job ) ;

} ;

class MessageSimObject : public Trick::SimObject {

    public:
        Trick::MessagePublisher mpublisher ;
        Trick::MessageCout mcout ;
        Trick::MessageThreadedCout mtcout ;
        Trick::MessageHSFile mfile ;
        Trick::MessageTCDevice mdevice ;
        Trick::PlaybackFile pfile ;
        Trick::MessageCustomManager message_file_manager ;

        MessageSimObject() {
            Trick::JobData * job __attribute__((unused)) ;
            job = this->add_job(0, 0, "default_data", NULL, 1, "mpublisher.subscribe", "", 60000) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 1, "default_data", NULL, 1, "mpublisher.subscribe", "", 60000) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 2, "default_data", NULL, 1, "mpublisher.subscribe", "", 60000) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 3, "default_data", NULL, 1, "mpublisher.subscribe", "", 60000) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 4, "default_data", NULL, 1, "mpublisher.subscribe", "", 60000) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 5, "default_data", NULL, 1, "mdevice.default_data", "", 60000) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 6, "initialization", NULL, 1, "mpublisher.init", "", 1) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 7, "initialization", NULL, 1, "mfile.init", "", 1) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 8, "initialization", NULL, 1, "pfile.init", "", 1) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 9, "initialization", NULL, 1, "mdevice.init", "", 1) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 10, "exec_time_tic_changed", NULL, 1, "mpublisher.init", "", 60000) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 11, "restart", NULL, 1, "mdevice.restart", "", 1) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 12, "restart", NULL, 1, "message_file_manager.restart", "", 1) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 13, "shutdown", NULL, 1, "mtcout.shutdown", "", 60000) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 14, "shutdown", NULL, 1, "mdevice.shutdown", "", 60000) ;
            job->add_tag("TRK") ;
        }

    public:
        virtual int call_function( Trick::JobData * curr_job ) ;
        virtual double call_function_double( Trick::JobData * curr_job ) ;


        //  Toggle log messages on separate thread on/off 

        void separate_thread_set_enabled(bool on_off) {
            if (on_off) {
                mpublisher.unsubscribe(&mcout);
                mpublisher.unsubscribe(&mdevice);
                mtcout.init();
                mpublisher.subscribe(&mtcout);
            } else {
                mpublisher.unsubscribe(&mtcout);
                mpublisher.subscribe(&mcout);
                mpublisher.subscribe(&mdevice);
            }
        }

    private:
        // This object is not copyable

        void operator =(const MessageSimObject &) {};
} ;

class JITSimObject : public Trick::SimObject {
    public:
        Trick::JITInputFile jit ;

        JITSimObject() {
            Trick::JobData * job __attribute__((unused)) ;
            job = this->add_job(0, 0, "input_processor", NULL, 1, "jit.process_sim_args", "", 50000) ;
            job = this->add_job(0, 1, "input_processor", NULL, 1, "jit.init", "", 50000) ;
        }

    public:
        virtual int call_function( Trick::JobData * curr_job ) ;
        virtual double call_function_double( Trick::JobData * curr_job ) ;

} ;

class InputProcessorSimObject : public Trick::SimObject {

    public:
        Trick::IPPython ip ;
        Trick::MTV mtv ;
        Trick::UnitsMap * units_map_ptr ; /* ** -- This is be ignored by ICG */

        InputProcessorSimObject() {
            Trick::JobData * job __attribute__((unused)) ;
units_map_ptr = Trick::UnitsMap::units_map() ;
            job = this->add_job(0, 0, "input_processor", NULL, 1, "ip.process_sim_args", "", 60000) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 1, "input_processor", NULL, 1, "ip.init", "", 60000) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 2, "restart", NULL, 1, "ip.restart", "", 60000) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 3, "shutdown", NULL, 1, "ip.shutdown", "", 65535) ;
            job->add_tag("TRK") ;

            Trick::IPPythonEvent::set_python_processor(&ip) ;
            Trick::IPPythonEvent::set_mtv(&mtv) ;
        }

        ~InputProcessorSimObject() {
            Trick::JobData * job __attribute__((unused)) ;
delete units_map_ptr ;
        }

    public:
        virtual int call_function( Trick::JobData * curr_job ) ;
        virtual double call_function_double( Trick::JobData * curr_job ) ;

} ;

class ThreadProcessEventSimObject : public Trick::SimObject {
    public:
        Trick::EventProcessor ep ;

        ThreadProcessEventSimObject(unsigned int thread_id ) {
            Trick::JobData * job __attribute__((unused)) ;
            job = this->add_job(0, 0, "initialization", NULL, 1, "ep.add_pending_events", "", 65535) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 1, "restart", NULL, 1, "ep.add_pending_events", "", 65535) ;
            job->add_tag("TRK") ;
            job = this->add_job(thread_id, 2, "top_of_frame", NULL, 1, "ep.add_pending_events", "", 60000) ;
            job->add_tag("TRK") ;
            job = this->add_job(thread_id, 3, "automatic", NULL, 1, "ep.process_event", "", 60000) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 4, "preload_checkpoint", NULL, 1, "ep.preload_checkpoint", "", 60000) ;
            job->add_tag("TRK") ;

            
            ep.set_process_event_job(get_job("ep.process_event")) ;
        }

    public:
        virtual int call_function( Trick::JobData * curr_job ) ;
        virtual double call_function_double( Trick::JobData * curr_job ) ;

} ;

class EventManagerSimObject : public Trick::SimObject {
    public:
        Trick::EventManager em ;
        Trick::EventProcessor ep ;
        std::vector< ThreadProcessEventSimObject * > thread_process_event_so ; // ** ignore for checkpointing

        std::vector<char *> thread_process_event_so_names ; // trick_io(**)


        void create_thread_process_event() {
            unsigned int ii ;
            unsigned int num_threads = exec_get_num_threads() ;
            for ( ii = 1 ; ii < num_threads ; ii++ ) {
                ThreadProcessEventSimObject * tpeso = new ThreadProcessEventSimObject(ii) ;
                std::ostringstream oss ;
                oss << "thread_process_event_" << ii ;
                thread_process_event_so.push_back(tpeso) ;
                char* tpe_name = strdup(oss.str().c_str()) ;
                thread_process_event_so_names.push_back(tpe_name) ;
                exec_add_sim_object(tpeso, tpe_name) ;
                TMM_declare_ext_var(tpeso, TRICK_STRUCTURED,"ThreadProcessEventSimObject", 0, tpe_name, 0, NULL) ;
                // Add the child thread event processor.

                em.add_event_processor(&(tpeso->ep)) ;
            }
        }

        EventManagerSimObject() {
            Trick::JobData * job __attribute__((unused)) ;
em.add_event_processor(&ep) ;
            job = this->add_job(0, 0, "default_data", NULL, 1, "create_thread_process_event", "", 60000) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 1, "initialization", NULL, 1, "ep.add_pending_events", "", 65535) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 2, "restart", NULL, 1, "ep.add_pending_events", "", 65535) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 3, "top_of_frame", NULL, 1, "ep.add_pending_events", "", 60000) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 4, "input_processor_run", NULL, 1, "ep.process_event", "", 60000) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 5, "exec_time_tic_changed", NULL, 1, "em.time_tic_changed", "", 60000) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 6, "preload_checkpoint", NULL, 1, "em.preload_checkpoint", "", 60000) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 7, "preload_checkpoint", NULL, 1, "ep.preload_checkpoint", "", 60000) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 8, "restart", NULL, 1, "em.restart", "", 60000) ;
            job->add_tag("TRK") ;

            
            ep.set_process_event_job(get_job("ep.process_event")) ;

        }

        ~EventManagerSimObject() {
            Trick::JobData * job __attribute__((unused)) ;
for(std::vector<char*>::iterator it = thread_process_event_so_names.begin(); it != thread_process_event_so_names.end(); ++it) {
                free(*it);
            }
        }

    public:
        virtual int call_function( Trick::JobData * curr_job ) ;
        virtual double call_function_double( Trick::JobData * curr_job ) ;

} ;

class VariableServerSimObject : public Trick::SimObject {

    public:
        Trick::VariableServer vs ;
        Trick::SimControlPanel sim_control_panel ;
        Trick::TrickView trick_view ;
        Trick::MalfunctionsTrickView malfunctions_trick_view ;
        Trick::StripChart stripchart ;

        Trick::MonteMonitor monte_monitor ;

        void monte_carlo_disable_all_apps() {
            if ( mc_get_slave_id() ) {
                Trick::remove_all_external_applications() ;
            }
        }


        VariableServerSimObject() {
            Trick::JobData * job __attribute__((unused)) ;
            job = this->add_job(0, 0, "default_data", NULL, 1, "vs.default_data", "", 60000) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 1, "initialization", NULL, 1, "vs.init", "", 0) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 2, "monte_master_init", NULL, 1, "vs.init", "", 60000) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 3, "initialization", NULL, 1, "monte_carlo_disable_all_apps", "", 1) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 4, "initialization", NULL, 1, "Trick::launch_all_external_applications", "", 1) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 5, "preload_checkpoint", NULL, 1, "vs.suspendPreCheckpointReload", "", 60000) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 6, "restart", NULL, 1, "vs.restart", "", 60000) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 7, "restart", NULL, 1, "vs.resumePostCheckpointReload", "", 60000) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 8, "top_of_frame", NULL, 1, "vs.copy_and_write_top", "", 60000) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 9, "automatic_last", NULL, 1, "vs.copy_and_write_scheduled", "", 60000) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 10, "freeze_init", NULL, 1, "vs.freeze_init", "", 60000) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 11, "freeze_automatic", NULL, 1, "vs.copy_and_write_freeze_scheduled", "", 60000) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 12, "freeze", NULL, 1, "vs.copy_and_write_freeze", "", 60000) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 13, "shutdown", NULL, 1, "vs.shutdown", "", 60000) ;
            job->add_tag("TRK") ;

            
            Trick::VariableServerSessionThread::set_vs_ptr(&vs) ;

            
            vs.set_copy_data_job(get_job("vs.copy_and_write_scheduled")) ;
            vs.set_copy_and_write_freeze_job(get_job("vs.copy_and_write_freeze_scheduled")) ;
        }

    public:
        virtual int call_function( Trick::JobData * curr_job ) ;
        virtual double call_function_double( Trick::JobData * curr_job ) ;


        private:

        VariableServerSimObject &operator=(const VariableServerSimObject &);
} ;

class DataRecordDispatcherSimObject : public Trick::SimObject {

    public:
        Trick::DataRecordDispatcher drd ;

        DataRecordDispatcherSimObject() {
            Trick::JobData * job __attribute__((unused)) ;
exec_register_scheduler(&drd) ;
            job = this->add_job(0, 0, "initialization", NULL, 1, "drd.remove_files", "", 1) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 1, "initialization", NULL, 1, "drd.init", "", 65534) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 2, "monte_slave_init", NULL, 1, "drd.remove_files", "", 60000) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 3, "end_of_frame", NULL, 1, "drd.signal_thread", "", 60000) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 4, "preload_checkpoint", NULL, 1, "drd.preload_checkpoint", "", 60000) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 5, "restart", NULL, 1, "drd.restart", "", 60000) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 6, "shutdown", NULL, 1, "drd.shutdown", "", 60000) ;
            job->add_tag("TRK") ;
        }

    public:
        virtual int call_function( Trick::JobData * curr_job ) ;
        virtual double call_function_double( Trick::JobData * curr_job ) ;


    private:
        void operator =(const DataRecordDispatcherSimObject &) ;
} ;

class RTSyncSimObject : public Trick::SimObject {

    public:

        Trick::GetTimeOfDayClock gtod_clock ;
        Trick::ITimer itimer ;
        Trick::RealtimeSync rt_sync ;

        RTSyncSimObject() : rt_sync(&gtod_clock, &itimer) {
            Trick::JobData * job __attribute__((unused)) ;
            job = this->add_job(0, 0, "default_data", NULL, 1, "rt_sync.get_sim_start_time", "", 0) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 1, "initialization", NULL, 1, "rt_sync.rt_clock->calc_sim_time_ratio", "", 65535) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 2, "initialization", NULL, 1, "rt_sync.initialize", "", 65535) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 3, "initialization", NULL, 1, "rt_sync.start_realtime", "", 65535) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 4, "initialization", NULL, 1, "rt_sync.get_sim_end_init_time", "", 65535) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 5, "restart", NULL, 1, "rt_sync.restart", "", 65535) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 6, "freeze_init", NULL, 1, "rt_sync.freeze_init", "", 60000) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 7, "freeze", NULL, 1, "rt_sync.freeze_pause", "", 65535) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 8, "unfreeze", NULL, 1, "rt_sync.unfreeze", "", 60000) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 9, "end_of_frame", NULL, 1, "rt_sync.rt_monitor", "", 65535) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 10, "shutdown", NULL, 1, "rt_sync.get_sim_end_time", "", 65535) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 11, "shutdown", NULL, 1, "rt_sync.shutdown", "", 65535) ;
            job->add_tag("TRK") ;
        }

    public:
        virtual int call_function( Trick::JobData * curr_job ) ;
        virtual double call_function_double( Trick::JobData * curr_job ) ;

} ;

class FrameLogSimObject : public Trick::SimObject {

    public:

        Trick::FrameLog frame_log ;

        FrameLogSimObject(Trick::Clock &in_clock) : frame_log(in_clock) {
            Trick::JobData * job __attribute__((unused)) ;
            job = this->add_job(0, 0, "instrumentation", NULL, 1, "frame_log.frame_clock_start", "", 0) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 1, "instrumentation", NULL, 1, "frame_log.frame_clock_stop", "", 65535) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 2, "default_data", NULL, 1, "frame_log.default_data", "", 60000) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 3, "initialization", NULL, 1, "frame_log.create_DP_files", "", 60000) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 4, "restart", NULL, 1, "frame_log.clear_data_record_info", "", 60000) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 5, "restart", NULL, 1, "frame_log.restart", "", 65535) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 6, "shutdown", NULL, 1, "frame_log.shutdown", "", 65535) ;
            job->add_tag("TRK") ;
        }

    public:
        virtual int call_function( Trick::JobData * curr_job ) ;
        virtual double call_function_double( Trick::JobData * curr_job ) ;


    private:
        // This object is not copyable

        void operator =(const FrameLogSimObject &) {};
} ;

class MasterSlaveSimObject : public Trick::SimObject {

    public:

        Trick::Master master ;
        Trick::Slave slave ;

        MasterSlaveSimObject() {
            Trick::JobData * job __attribute__((unused)) ;
            job = this->add_job(0, 0, "initialization", NULL, 1, "master.process_sim_args", "", 0) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 1, "initialization", NULL, 1, "slave.process_sim_args", "", 0) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 2, "initialization", NULL, 1, "master.init", "", 0) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 3, "initialization", NULL, 1, "slave.init", "", 0) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 4, "checkpoint", NULL, 1, "master.checkpoint", "", 60000) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 5, "preload_checkpoint", NULL, 1, "master.preload_checkpoint", "", 60000) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 6, "end_of_frame", NULL, 1, "master.end_of_frame_status_from_slave", "", 65534) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 7, "end_of_frame", NULL, 1, "master.end_of_frame_status_to_slave", "", 65535) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 8, "end_of_frame", NULL, 1, "slave.end_of_frame", "", 65534) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 9, "freeze_init", NULL, 1, "master.freeze_init", "", 65535) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 10, "freeze_init", NULL, 1, "slave.freeze_init", "", 65535) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 11, "freeze", NULL, 1, "master.freeze", "", 65535) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 12, "freeze", NULL, 1, "slave.freeze", "", 65535) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 13, "unfreeze", NULL, 1, "master.unfreeze", "", 65535) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 14, "unfreeze", NULL, 1, "slave.unfreeze", "", 65535) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 15, "shutdown", NULL, 1, "master.shutdown", "", 65534) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 16, "shutdown", NULL, 1, "slave.shutdown", "", 65534) ;
            job->add_tag("TRK") ;

        }

    public:
        virtual int call_function( Trick::JobData * curr_job ) ;
        virtual double call_function_double( Trick::JobData * curr_job ) ;

} ;

class InstrumentationSimObject : public Trick::SimObject {

    public:
        Trick::EchoJobs echo_jobs ;
        Trick::DebugPause debug_pause ;

        InstrumentationSimObject() {
            Trick::JobData * job __attribute__((unused)) ;
            job = this->add_job(0, 0, "instrumentation", NULL, 1, "echo_jobs.echo_job", "", 60000) ;
            job->add_tag("TRK") ;
            job = this->add_job(0, 1, "instrumentation", NULL, 1, "debug_pause.debug_pause", "", 60000) ;
            job->add_tag("TRK") ;
        }

    public:
        virtual int call_function( Trick::JobData * curr_job ) ;
        virtual double call_function_double( Trick::JobData * curr_job ) ;

} ;

class InjectorExecSimObject : public Trick::SimObject {
    public:
        Trick::RtiExec rtie ;
        InjectorExecSimObject(unsigned int thread_id ) {
            Trick::JobData * job __attribute__((unused)) ;
            job = this->add_job(thread_id, 0, "top_of_frame", NULL, 1, "rtie.Exec", "", 60000) ;
            job->add_tag("TRK") ;
        }

    public:
        virtual int call_function( Trick::JobData * curr_job ) ;
        virtual double call_function_double( Trick::JobData * curr_job ) ;

} ;

class InjectorSimObject : public Trick::SimObject {

    public:
        Trick::RtiStager rtis ;
        std::vector< InjectorExecSimObject * > injector_executor_so ; // trick_io(**)

        std::vector<char *> injector_executor_so_names ; // trick_io(**)


        void create_injector_executors() {
            unsigned int ii ;
            unsigned int num_threads = exec_get_num_threads() ;
            for ( ii = 0 ; ii < num_threads ; ii++ ) {
                InjectorExecSimObject * ieso = new InjectorExecSimObject(ii) ;
                std::ostringstream oss ;
                oss << "trick_injector_executor_" << ii ;
                injector_executor_so.push_back(ieso) ;
                char* injector_name = strdup(oss.str().c_str()) ;
                injector_executor_so_names.push_back(injector_name);
                exec_add_sim_object(ieso, injector_name) ;
                TMM_declare_ext_var(ieso, TRICK_STRUCTURED,"InjectorExecSimObject", 0, injector_name, 0, NULL) ;
                // Add the child thread realtime injector.

                rtis.AddInjectorExecutor(&(ieso->rtie)) ;
            }
        }

        InjectorSimObject() {
            Trick::JobData * job __attribute__((unused)) ;
            job = this->add_job(0, 0, "default_data", NULL, 1, "create_injector_executors", "", 0) ;
            job->add_tag("TRK") ;
        }

        ~InjectorSimObject() {
            Trick::JobData * job __attribute__((unused)) ;
for(std::vector<char*>::iterator it = injector_executor_so_names.begin(); it != injector_executor_so_names.end(); ++it) {
            free(*it);
    }
        }

    public:
        virtual int call_function( Trick::JobData * curr_job ) ;
        virtual double call_function_double( Trick::JobData * curr_job ) ;

} ;

class ZeroconfSimObject : public Trick::SimObject {
    public:
        Trick::Zeroconf zc ;
        ZeroconfSimObject() {
            Trick::JobData * job __attribute__((unused)) ;
            job = this->add_job(0, 0, "initialization", NULL, 1, "zc.init", "", 60000) ;
            job->add_tag("TRK") ;
        }

    public:
        virtual int call_function( Trick::JobData * curr_job ) ;
        virtual double call_function_double( Trick::JobData * curr_job ) ;

} ;

class UnitTestSimObject : public Trick::SimObject {
    public:
        Trick::UnitTest unit_tests ;
        UnitTestSimObject() {
            Trick::JobData * job __attribute__((unused)) ;
            job = this->add_job(0, 0, "shutdown", NULL, 1, "unit_tests.write_output", "", 65535) ;
        }

    public:
        virtual int call_function( Trick::JobData * curr_job ) ;
        virtual double call_function_double( Trick::JobData * curr_job ) ;

} ;

class UdUnitsSimObject : public Trick::SimObject {
    public:
        Trick::UdUnits udunits ;
        UdUnitsSimObject() {
            Trick::JobData * job __attribute__((unused)) ;
            job = this->add_job(0, 0, "default_data", NULL, 1, "udunits.read_default_xml", "", 65535) ;
        }

    public:
        virtual int call_function( Trick::JobData * curr_job ) ;
        virtual double call_function_double( Trick::JobData * curr_job ) ;

} ;

class TRACESimObject : public Trick::SimObject {
    public:
    BALL ball ;
    TRACESimObject() {
            Trick::JobData * job __attribute__((unused)) ;
            job = this->add_job(0, 0, "initialization", NULL, 1, "robot_init", "", 60000) ;
            job = this->add_job(0, 1, "default_data", NULL, 1, "robot_default_data", "", 60000) ;
            job = this->add_job(0, 2, "derivative", NULL, 1, "robot_deriv", "", 60000) ;
            job = this->add_job(0, 3, "integration", NULL, 1, "robot_integ", "", 60000) ;
            job = this->add_job(0, 4, "shutdown", NULL, 1, "robot_shutdown", "", 60000) ;
    }

    public:
        virtual int call_function( Trick::JobData * curr_job ) ;
        virtual double call_function_double( Trick::JobData * curr_job ) ;

} ;

#ifdef TRICK_ICG
#endif

#ifndef SWIG
extern SysSimObject trick_sys ;
extern MonteCarloSimObject trick_mc ;
extern MemoryManagerSimObject trick_mm ;
extern CheckPointRestartSimObject trick_cpr ;
extern SieSimObject trick_sie ;
extern CommandLineArgumentsSimObject trick_cmd_args ;
extern MessageSimObject trick_message ;
extern JITSimObject trick_jit ;
extern InputProcessorSimObject trick_ip ;
extern EventManagerSimObject trick_em ;
extern VariableServerSimObject trick_vs ;
extern DataRecordDispatcherSimObject trick_data_record ;
extern RTSyncSimObject trick_real_time ;
extern FrameLogSimObject trick_frame_log;
extern MasterSlaveSimObject trick_master_slave ;
extern InstrumentationSimObject trick_instruments ;
extern InjectorSimObject trick_inject ;
extern ZeroconfSimObject trick_zero_conf ;
extern UnitTestSimObject trick_utest ;
extern UdUnitsSimObject trick_udunits;
extern TRACESimObject dyn ;
#endif


#endif

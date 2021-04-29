#ifndef STATE_MACHINE_H_
#define STATE_MACHINE_H_
#include <mutex>

class StateMachine
{
public:
    StateMachine()
    {
        state = State_SystemIdle;
    }

    enum State
    {
        State_SystemIdle = 0,

        State_SuspendTrack = 1,    //暂停追踪
        State_VertexTracking = 2,  //顶点型路径追踪中
        State_CurveTracking = 3,   //连续型路径追踪中
        State_CompleteTrack = 4,   //完成追踪

        State_SystemBusy = 6,      //系统忙，正在处理请求指令

        State_CurvePathRecording = 0xA,  //连续型路径记录中
        State_VertexPathRecording = 0xB, //顶点型路径记录中
    };

    bool isBusy()
    {
        return (state == State_SystemBusy);
    }

    bool isRecording()
    {
        if(state == State_CurvePathRecording ||
            state == State_VertexPathRecording)
            return true;
        return false;
    }

    bool isTracking()
    {
        if(state >= State_SuspendTrack && 
           state <= State_CurveTracking)
            return true;
        return false;
    }

    bool isIdle()
    {
        return state == State_SystemIdle;
    }

    void set(uint8_t _state)
    {
        mutex.lock();
        state = _state;
        mutex.unlock();
    }

    uint8_t get() const
    {
        return state;
    }
    
private:
    uint8_t state;
    std::mutex mutex;
};

#endif

constexpr AxisInput AxisSample(int16_t position, uint32_t at) {
    return AxisInput{position, at, true, AxisLimits{true, false, false}};
}

constexpr AxisAck Ack(const AxisControl& axis, uint32_t at) {
    return AxisAck{axis.Status().demand, at, true};
}

constexpr AxisControl Idle() {
    AxisControl axis;
    axis.Configure(AxisConfig());
    axis.Step(false, 0, AxisSample(0, 0), AxisAck{}, 0);
    axis.Step(false, 0, AxisSample(0, 1), Ack(axis, 1), 1);
    axis.Step(false, 0, AxisSample(0, 11), Ack(axis, 11), 11);
    return axis;
}

constexpr AxisControl Moving() {
    AxisControl axis = Idle();
    axis.Step(true, 80, AxisSample(0, 12), Ack(axis, 12), 12);
    axis.Step(true, 80, AxisSample(0, 13), Ack(axis, 13), 13);
    return axis;
}

constexpr bool StartupProof() {
    AxisControl axis;
    if (!axis.Configure(AxisConfig())) return false;
    if (axis.Step(true, 80, AxisSample(0, 0), AxisAck{}, 0).demand.direction != 0) return false;
    const AxisStatus result = axis.Step(true, 80, AxisSample(0, 15), AxisAck{}, 15);
    return result.state == AxisState::Fault && result.faults == static_cast<uint32_t>(AxisFault::StopTimeout);
}

constexpr bool StartProof() {
    AxisControl axis = Idle();
    const AxisAck old = Ack(axis, 12);
    const AxisStatus start = axis.Step(true, 80, AxisSample(0, 12), old, 12);
    if (start.state != AxisState::Starting || start.demand.direction != 1) return false;
    AxisAck wrong = old;
    wrong.at_ms = 27;
    const AxisStatus result = axis.Step(true, 80, AxisSample(0, 27), wrong, 27);
    return result.state == AxisState::Fault && result.demand.direction == 0 && result.faults == static_cast<uint32_t>(AxisFault::StartTimeout);
}

constexpr bool ReverseProof() {
    AxisControl axis = Moving();
    const AxisAck positive = Ack(axis, 14);
    if (axis.Step(true, -80, AxisSample(0, 14), positive, 14).demand.direction != 0) return false;
    axis.Step(true, -80, AxisSample(0, 15), positive, 15);
    if (axis.Status().state != AxisState::Stopping) return false;
    axis.Step(true, -80, AxisSample(0, 20), Ack(axis, 20), 20);
    if (axis.Step(true, -80, AxisSample(0, 29), Ack(axis, 29), 29).demand.direction != 0) return false;
    axis.Step(true, -80, AxisSample(0, 30), Ack(axis, 30), 30);
    return axis.Step(true, -80, AxisSample(0, 31), Ack(axis, 31), 31).demand.direction == -1;
}

constexpr bool StopProof() {
    AxisControl axis = Moving();
    const AxisAck old = Ack(axis, 14);
    axis.Step(false, 80, AxisSample(0, 14), old, 14);
    const AxisStatus result = axis.Step(false, 80, AxisSample(0, 29), old, 29);
    return result.demand.direction == 0 && result.faults == static_cast<uint32_t>(AxisFault::StopTimeout);
}

constexpr bool TravelBudget() {
    AxisControl axis = Moving();
    for (uint32_t at = 18; at < 112; at += 10) {
        const int16_t position = static_cast<int16_t>((at - 18) / 10);
        if (axis.Step(true, 80, AxisSample(position, at), Ack(axis, at), at).faults != 0) return false;
    }
    const AxisStatus result = axis.Step(true, 80, AxisSample(10, 112), Ack(axis, 112), 112);
    return result.demand.direction == 0 && result.faults == static_cast<uint32_t>(AxisFault::TravelTimeout);
}

constexpr bool NoProgress() {
    AxisControl axis = Moving();
    axis.Step(true, 80, AxisSample(0, 18), Ack(axis, 18), 18);
    // 重用旧采样不能被当成一个新的无位移测量。
    if (axis.Step(true, 80, AxisSample(0, 18), Ack(axis, 38), 38).faults != 0) return false;
    return axis.Step(true, 80, AxisSample(0, 39), Ack(axis, 39), 39).faults == static_cast<uint32_t>(AxisFault::NoProgress);
}

constexpr bool WrongDirection() {
    AxisControl axis = Moving();
    axis.Step(true, 80, AxisSample(0, 18), Ack(axis, 18), 18);
    return axis.Step(true, 80, AxisSample(-4, 19), Ack(axis, 19), 19).faults == static_cast<uint32_t>(AxisFault::WrongDirection);
}

constexpr bool TravelLimits() {
    AxisControl hard = Idle();
    AxisInput input = AxisSample(0, 12);
    input.limits.upper = true;
    const AxisStatus h = hard.Step(true, 80, input, Ack(hard, 12), 12);
    AxisControl soft = Idle();
    const AxisStatus s = soft.Step(true, 101, AxisSample(0, 12), Ack(soft, 12), 12);
    return h.demand.direction == 0 && h.faults == static_cast<uint32_t>(AxisFault::Limit) && s.faults == static_cast<uint32_t>(AxisFault::Limit);
}

constexpr bool Hysteresis() {
    AxisControl axis = Idle();
    if (axis.Step(true, 3, AxisSample(0, 12), Ack(axis, 12), 12).demand.direction != 0) return false;
    if (axis.Step(true, 4, AxisSample(0, 13), Ack(axis, 13), 13).demand.direction != 1) return false;
    axis.Step(true, 4, AxisSample(2, 14), Ack(axis, 14), 14);
    if (axis.Status().demand.direction != 1) return false;
    return axis.Step(true, 4, AxisSample(3, 15), Ack(axis, 15), 15).demand.direction == 0;
}

constexpr bool StaleOutput() {
    AxisControl axis = Moving();
    return axis.Step(true, 80, AxisSample(0, 29), Ack(axis, 13), 29).faults == static_cast<uint32_t>(AxisFault::FeedbackLost);
}

constexpr bool SecondaryFault() {
    AxisControl axis = Moving();
    axis.Step(true, 80, AxisSample(0, 18), Ack(axis, 18), 18);
    const AxisAck old = Ack(axis, 19);
    axis.Step(true, 80, AxisSample(-4, 19), old, 19);
    const AxisStatus result = axis.Step(false, 80, AxisSample(-4, 35), old, 35);
    return result.faults == (static_cast<uint32_t>(AxisFault::WrongDirection) | static_cast<uint32_t>(AxisFault::StopTimeout));
}

static_assert(StartupProof(), "startup without safe output proof");
static_assert(StartProof(), "old acknowledgement accepted for new output");
static_assert(ReverseProof(), "reversal without confirmed neutral interval");
static_assert(StopProof(), "missing stop acknowledgement ignored");
static_assert(TravelBudget(), "periodic target reset movement budget");
static_assert(NoProgress(), "stalled motion not detected or stale sample reused");
static_assert(WrongDirection(), "wrong direction not detected");
static_assert(TravelLimits(), "travel limit ignored");
static_assert(Hysteresis(), "start and stop thresholds not separated");
static_assert(StaleOutput(), "stale execution feedback used");
static_assert(SecondaryFault(), "motion fault concealed output stop failure");

constexpr bool ResetDelay() {
    AxisControl axis = Moving();
    AxisInput input = AxisSample(0, 14);
    input.limits.upper = true;
    axis.Step(true, 80, input, Ack(axis, 14), 14);
    if (!axis.CanReset(Ack(axis, 15), 15)) return false;
    axis.Reset(15);
    if (axis.Step(true, -80, AxisSample(0, 24), Ack(axis, 24), 24).demand.direction != 0) return false;
    axis.Step(true, -80, AxisSample(0, 25), Ack(axis, 25), 25);
    return axis.Step(true, -80, AxisSample(0, 26), Ack(axis, 26), 26).demand.direction == -1;
}

constexpr bool InvalidInput() {
    AxisControl axis = Moving();
    AxisInput input = AxisSample(0, 14);
    input.limits.lower = true;
    input.limits.upper = true;
    const AxisStatus result = axis.Step(true, 0, input, Ack(axis, 14), 14);
    return result.demand.direction == 0 && result.faults == static_cast<uint32_t>(AxisFault::Input);
}

static_assert(ResetDelay(), "fault reset bypassed neutral interval");
static_assert(InvalidInput(), "contradictory limit inputs ignored");

constexpr bool LateConfirmation() {
    AxisControl axis = Idle();
    axis.Step(true, 80, AxisSample(0, 12), Ack(axis, 12), 12);
    const AxisStatus start = axis.Step(true, 80, AxisSample(0, 28), Ack(axis, 28), 28);
    AxisControl stop = Moving();
    stop.Step(false, 80, AxisSample(0, 14), Ack(stop, 14), 14);
    const AxisStatus end = stop.Step(false, 80, AxisSample(0, 30), Ack(stop, 30), 30);
    return start.faults == static_cast<uint32_t>(AxisFault::StartTimeout) && end.faults == static_cast<uint32_t>(AxisFault::StopTimeout);
}

static_assert(LateConfirmation(), "late acknowledgement concealed timeout");

package org.ironmaple.simulation.physics.threading;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.ironmaple.simulation.physics.PhysicsBody;
import org.ironmaple.simulation.physics.jolt.JoltBody;

/**
 *
 *
 * <h1>Thread-Safe Jolt Body Wrapper</h1>
 *
 * <p>Wraps a {@link JoltBody} to provide thread-safe access from the main thread.
 *
 * <p>Reads are served from cached state snapshots, writes are queued for the physics thread.
 */
public class ThreadedJoltBody implements PhysicsBody {
    private final JoltBody realBody;
    private final ThreadedPhysicsProxy proxy;

    /**
     *
     *
     * <h2>Creates a Thread-Safe Wrapper.</h2>
     *
     * @param realBody the underlying Jolt body
     * @param proxy the proxy for thread-safe communication
     */
    public ThreadedJoltBody(JoltBody realBody, ThreadedPhysicsProxy proxy) {
        this.realBody = realBody;
        this.proxy = proxy;

        // Link wrapper to real body for lookups
        realBody.setUserData(this);
    }

    /**
     *
     *
     * <h2>Gets the Underlying Jolt Body.</h2>
     *
     * @return the real Jolt body
     */
    public JoltBody getRealBody() {
        return realBody;
    }

    /**
     *
     *
     * <h2>Gets the Jolt Body Object.</h2>
     *
     * @return the Jolt Body
     */
    public com.github.stephengold.joltjni.Body getBody() {
        return realBody.getBody();
    }

    /**
     *
     *
     * <h2>Gets the Tracking ID.</h2>
     *
     * @return the unique tracking ID
     */
    public int getTrackingId() {
        return realBody.getTrackingId();
    }

    @Override
    public int getBodyId() {
        return getTrackingId();
    }

    @Override
    public Pose3d getPose3d() {
        SimulationState state = proxy.getLatestState();
        SimulationState.BodyState bodyState = state.getBodyState(realBody.getTrackingId());
        if (bodyState != null) {
            return bodyState.pose();
        }
        // Fallback to real (may be stale)
        return realBody.getPose3d();
    }

    @Override
    public void setPose3d(Pose3d pose) {
        proxy.queueBodyPoseUpdate(realBody.getTrackingId(), pose);
    }

    @Override
    public Translation3d getLinearVelocityMPS() {
        SimulationState state = proxy.getLatestState();
        SimulationState.BodyState bodyState = state.getBodyState(realBody.getTrackingId());
        if (bodyState != null) {
            return bodyState.linearVelocity();
        }
        return realBody.getLinearVelocityMPS();
    }

    @Override
    public void setLinearVelocityMPS(Translation3d velocityMPS) {
        proxy.queueBodyVelocityUpdate(realBody.getTrackingId(), velocityMPS, null);
    }

    @Override
    public Translation3d getAngularVelocityRadPerSec() {
        SimulationState state = proxy.getLatestState();
        SimulationState.BodyState bodyState = state.getBodyState(realBody.getTrackingId());
        if (bodyState != null) {
            return bodyState.angularVelocity();
        }
        return realBody.getAngularVelocityRadPerSec();
    }

    @Override
    public void setAngularVelocityRadPerSec(Translation3d angularVelocityRadPerSec) {
        proxy.queueBodyVelocityUpdate(realBody.getTrackingId(), null, angularVelocityRadPerSec);
    }

    @Override
    public void applyForce(Translation3d forceNewtons) {
        proxy.queueBodyForce(realBody.getTrackingId(), forceNewtons, null);
    }

    @Override
    public void applyForceAtPoint(Translation3d forceNewtons, Translation3d pointWorld) {
        proxy.queueBodyForce(realBody.getTrackingId(), forceNewtons, pointWorld);
    }

    @Override
    public void applyTorque(Translation3d torqueNewtonMeters) {
        proxy.queueBodyTorque(realBody.getTrackingId(), torqueNewtonMeters);
    }

    @Override
    public double getMassKg() {
        return realBody.getMassKg();
    }

    @Override
    public boolean isStatic() {
        return realBody.isStatic();
    }

    @Override
    public void setFriction(double friction) {
        realBody.setFriction(friction);
    }

    @Override
    public void setRestitution(double restitution) {
        realBody.setRestitution(restitution);
    }

    @Override
    public Translation3d getLinearVelocityAtPointMPS(Translation3d pointWorld) {
        // Calculate from cached state
        Pose3d pose = getPose3d();
        Translation3d linearVel = getLinearVelocityMPS();
        Translation3d angularVel = getAngularVelocityRadPerSec();

        // r = point - center
        Translation3d r = pointWorld.minus(pose.getTranslation());

        // v_point = v_com + omega x r
        double tx = angularVel.getY() * r.getZ() - angularVel.getZ() * r.getY();
        double ty = angularVel.getZ() * r.getX() - angularVel.getX() * r.getZ();
        double tz = angularVel.getX() * r.getY() - angularVel.getY() * r.getX();

        return linearVel.plus(new Translation3d(tx, ty, tz));
    }

    @Override
    public void setUserData(Object data) {
        // User data is managed separately for wrapper
        // The real body's userData is reserved for linking
        proxy.setBodyUserData(realBody.getTrackingId(), data);
    }

    @Override
    public Object getUserData() {
        return proxy.getBodyUserData(realBody.getTrackingId());
    }

    @Override
    public void setDamping(double linearDamping, double angularDamping) {
        proxy.queueBodyDamping(realBody.getTrackingId(), linearDamping, angularDamping);
    }

    @Override
    public void setContactReporting(boolean enable) {
        realBody.setContactReporting(enable);
    }

    @Override
    public void setCollisionLayer(int layer) {
        // Direct call acceptable for now, or queue if thread safety critical for layer
        // switch
        // Given Jolt's locking, direct call might be okay if it handles its own locks,
        // but Jolt generally requires body interface access which is thread-safe.
        // realBody uses bodyInterface.setObjectLayer which is thread-safe in Jolt.
        realBody.setCollisionLayer(layer);
    }
}

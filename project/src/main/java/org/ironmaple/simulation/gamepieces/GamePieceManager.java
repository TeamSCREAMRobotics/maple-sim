package org.ironmaple.simulation.gamepieces;

import edu.wpi.first.math.geometry.Pose3d;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.UUID;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.stream.Collectors;

/**
 *
 *
 * <h1>Unified Game Piece Manager</h1>
 *
 * <p>The single source of truth for all game piece management in the simulation. Handles:
 *
 * <ul>
 *   <li>Storage of all active pieces
 *   <li>Lifecycle (spawn, transitions, destruction)
 *   <li>Ownership tracking (field vs intake vs shooter vs goal)
 *   <li>Pose queries for rendering
 *   <li>Custom user lifecycle hooks
 * </ul>
 */
public class GamePieceManager {

    // Master registry - all pieces have a unique ID
    private final Map<UUID, ManagedGamePiece> allPieces = new ConcurrentHashMap<>();

    // Callbacks
    private final List<BiConsumer<ManagedGamePiece, GamePieceState>> stateChangeCallbacks =
            new CopyOnWriteArrayList<>();
    private final List<Consumer<ManagedGamePiece>> scoredCallbacks = new CopyOnWriteArrayList<>();
    private final List<Consumer<ManagedGamePiece>> spawnedCallbacks = new CopyOnWriteArrayList<>();

    // Legacy pose sources for custom rendering
    private final List<GamePiecePoseSource> poseSources = new CopyOnWriteArrayList<>();

    /** Functional interface for custom pose sources. */
    @FunctionalInterface
    public interface GamePiecePoseSource {
        void addPosesToList(String type, List<Pose3d> poseList);
    }

    // ===== SPAWNING =====

    /**
     * Spawns a game piece on the field.
     *
     * @param piece the physics-backed game piece
     * @return the UUID of the managed piece
     */
    public UUID spawnOnField(GamePiece piece) {
        ManagedGamePiece managed = new ManagedGamePiece(piece.getType(), GamePieceState.ON_FIELD, piece);
        allPieces.put(managed.getId(), managed);
        notifySpawned(managed);
        return managed.getId();
    }

    /**
     * Spawns a game piece in flight (projectile).
     *
     * @param projectile the projectile piece
     * @return the UUID of the managed piece
     */
    public UUID spawnInFlight(GamePiece projectile) {
        ManagedGamePiece managed = new ManagedGamePiece(projectile.getType(), GamePieceState.IN_FLIGHT, projectile);
        allPieces.put(managed.getId(), managed);
        notifySpawned(managed);
        return managed.getId();
    }

    /**
     * Creates a virtual game piece (no physics backing) in a specified state.
     *
     * @param type the piece type
     * @param state the initial state
     * @param owner the owner object
     * @return the UUID of the managed piece
     */
    public UUID createVirtual(String type, GamePieceState state, Object owner) {
        ManagedGamePiece managed = new ManagedGamePiece(type, state, null);
        managed.setOwner(owner);
        allPieces.put(managed.getId(), managed);
        notifySpawned(managed);
        return managed.getId();
    }

    // ===== TRANSITIONS =====

    /**
     * Transitions a piece to the intake state.
     *
     * @param pieceId the piece UUID
     * @param intake the intake simulation owner
     */
    public void transitionToIntake(UUID pieceId, Object intake) {
        ManagedGamePiece piece = allPieces.get(pieceId);
        if (piece == null) return;
        GamePieceState oldState = piece.getState();
        piece.setState(GamePieceState.IN_INTAKE);
        piece.setOwner(intake);
        piece.setUnderlying(null); // No longer physics-backed
        notifyStateChange(piece, oldState);
    }

    /**
     * Transitions a piece to the shooter state.
     *
     * @param pieceId the piece UUID
     * @param shooter the shooter simulation owner
     */
    public void transitionToShooter(UUID pieceId, Object shooter) {
        ManagedGamePiece piece = allPieces.get(pieceId);
        if (piece == null) return;
        GamePieceState oldState = piece.getState();
        piece.setState(GamePieceState.IN_SHOOTER);
        piece.setOwner(shooter);
        notifyStateChange(piece, oldState);
    }

    /**
     * Transitions a piece to in-flight state.
     *
     * @param pieceId the piece UUID
     * @param projectile the projectile physics object
     */
    public void transitionToFlight(UUID pieceId, GamePiece projectile) {
        ManagedGamePiece piece = allPieces.get(pieceId);
        if (piece == null) return;
        GamePieceState oldState = piece.getState();
        piece.setState(GamePieceState.IN_FLIGHT);
        piece.setOwner(null);
        piece.setUnderlying(projectile);
        notifyStateChange(piece, oldState);
    }

    /**
     * Transitions a piece to scored in goal.
     *
     * @param pieceId the piece UUID
     * @param goal the goal that scored it
     */
    public void transitionToGoal(UUID pieceId, Object goal) {
        ManagedGamePiece piece = allPieces.get(pieceId);
        if (piece == null) return;
        GamePieceState oldState = piece.getState();
        piece.setState(GamePieceState.IN_GOAL);
        piece.setOwner(goal);
        piece.setUnderlying(null);
        notifyStateChange(piece, oldState);
        notifyScored(piece);
    }

    /**
     * Transitions a piece back to on-field state (e.g., after landing).
     *
     * @param pieceId the piece UUID
     * @param fieldPiece the new physics object
     */
    public void transitionToField(UUID pieceId, GamePiece fieldPiece) {
        ManagedGamePiece piece = allPieces.get(pieceId);
        if (piece == null) return;
        GamePieceState oldState = piece.getState();
        piece.setState(GamePieceState.ON_FIELD);
        piece.setOwner(null);
        piece.setUnderlying(fieldPiece);
        notifyStateChange(piece, oldState);
    }

    // ===== DESTRUCTION =====

    /**
     * Removes a piece from the manager.
     *
     * @param pieceId the UUID to remove
     * @return the removed piece, or null if not found
     */
    public ManagedGamePiece remove(UUID pieceId) {
        return allPieces.remove(pieceId);
    }

    /** Clears all managed pieces. */
    public void clearAll() {
        allPieces.clear();
    }

    /**
     * Clears all pieces in a specific state.
     *
     * @param state the state to clear
     */
    public void clearByState(GamePieceState state) {
        allPieces.entrySet().removeIf(e -> e.getValue().getState() == state);
    }

    // ===== QUERIES =====

    /**
     * Gets all managed pieces.
     *
     * @return list of all pieces
     */
    public List<ManagedGamePiece> getAll() {
        return new ArrayList<>(allPieces.values());
    }

    /**
     * Gets pieces by type.
     *
     * @param type the piece type
     * @return list of matching pieces
     */
    public List<ManagedGamePiece> getByType(String type) {
        return allPieces.values().stream()
                .filter(p -> Objects.equals(p.getType(), type))
                .collect(Collectors.toList());
    }

    /**
     * Gets pieces by state.
     *
     * @param state the state to filter
     * @return list of matching pieces
     */
    public List<ManagedGamePiece> getByState(GamePieceState state) {
        return allPieces.values().stream().filter(p -> p.getState() == state).collect(Collectors.toList());
    }

    /**
     * Gets pieces owned by a specific object.
     *
     * @param owner the owner object
     * @return list of matching pieces
     */
    public List<ManagedGamePiece> getByOwner(Object owner) {
        return allPieces.values().stream()
                .filter(p -> Objects.equals(p.getOwner(), owner))
                .collect(Collectors.toList());
    }

    /**
     * Counts pieces by type.
     *
     * @param type the piece type
     * @return count
     */
    public int countByType(String type) {
        return (int) allPieces.values().stream()
                .filter(p -> Objects.equals(p.getType(), type))
                .count();
    }

    /**
     * Counts pieces by state.
     *
     * @param state the state to count
     * @return count
     */
    public int countByState(GamePieceState state) {
        return (int)
                allPieces.values().stream().filter(p -> p.getState() == state).count();
    }

    /**
     * Counts pieces owned by a specific object.
     *
     * @param owner the owner
     * @return count
     */
    public int countByOwner(Object owner) {
        return (int) allPieces.values().stream()
                .filter(p -> Objects.equals(p.getOwner(), owner))
                .count();
    }

    /**
     * Gets a piece by UUID.
     *
     * @param id the UUID
     * @return the piece, or null if not found
     */
    public ManagedGamePiece getById(UUID id) {
        return allPieces.get(id);
    }

    /**
     * Finds pieces matching a type and state.
     *
     * @param type the piece type
     * @param state the state
     * @return list of matching pieces
     */
    public List<ManagedGamePiece> findByTypeAndState(String type, GamePieceState state) {
        return allPieces.values().stream()
                .filter(p -> Objects.equals(p.getType(), type) && p.getState() == state)
                .collect(Collectors.toList());
    }

    // ===== POSE QUERIES (for rendering) =====

    /**
     * Gets poses of all pieces by type (from managed pieces + legacy sources).
     *
     * @param type the piece type
     * @return list of poses
     */
    public List<Pose3d> getPosesByType(String type) {
        List<Pose3d> result = allPieces.values().stream()
                .filter(p -> Objects.equals(p.getType(), type))
                .map(ManagedGamePiece::getPose)
                .filter(Objects::nonNull)
                .collect(Collectors.toList());

        // Add from legacy pose sources
        for (GamePiecePoseSource source : poseSources) {
            source.addPosesToList(type, result);
        }
        return result;
    }

    /**
     * Gets poses of pieces in a specific state.
     *
     * @param state the state
     * @return list of poses
     */
    public List<Pose3d> getPosesByState(GamePieceState state) {
        return allPieces.values().stream()
                .filter(p -> p.getState() == state)
                .map(ManagedGamePiece::getPose)
                .filter(Objects::nonNull)
                .collect(Collectors.toList());
    }

    /**
     * Gets all poses as array by type.
     *
     * @param type the piece type
     * @return array of poses
     */
    public Pose3d[] getPosesArrayByType(String type) {
        return getPosesByType(type).toArray(Pose3d[]::new);
    }

    // ===== LEGACY POSE SOURCE SUPPORT =====

    /**
     * Registers a legacy pose source.
     *
     * @param source the pose source
     */
    public void registerPoseSource(GamePiecePoseSource source) {
        if (!poseSources.contains(source)) {
            poseSources.add(source);
        }
    }

    /**
     * Unregisters a legacy pose source.
     *
     * @param source the pose source
     */
    public void unregisterPoseSource(GamePiecePoseSource source) {
        poseSources.remove(source);
    }

    /**
     * Legacy method for adding poses to list.
     *
     * @param type the type
     * @param result the list to add to
     */
    public void getAllPosesByType(String type, List<Pose3d> result) {
        result.addAll(getPosesByType(type));
    }

    /** Legacy method. */
    public List<Pose3d> getAllPosesByType(String type) {
        return getPosesByType(type);
    }

    // ===== CALLBACKS =====

    /**
     * Registers a callback for state changes.
     *
     * @param callback receives (piece, oldState)
     */
    public void onStateChange(BiConsumer<ManagedGamePiece, GamePieceState> callback) {
        stateChangeCallbacks.add(callback);
    }

    /**
     * Registers a callback for when a piece is scored.
     *
     * @param callback receives the scored piece
     */
    public void onScored(Consumer<ManagedGamePiece> callback) {
        scoredCallbacks.add(callback);
    }

    /**
     * Registers a callback for when a piece is spawned.
     *
     * @param callback receives the spawned piece
     */
    public void onSpawned(Consumer<ManagedGamePiece> callback) {
        spawnedCallbacks.add(callback);
    }

    private void notifyStateChange(ManagedGamePiece piece, GamePieceState oldState) {
        for (BiConsumer<ManagedGamePiece, GamePieceState> cb : stateChangeCallbacks) {
            cb.accept(piece, oldState);
        }
    }

    private void notifyScored(ManagedGamePiece piece) {
        for (Consumer<ManagedGamePiece> cb : scoredCallbacks) {
            cb.accept(piece);
        }
    }

    private void notifySpawned(ManagedGamePiece piece) {
        for (Consumer<ManagedGamePiece> cb : spawnedCallbacks) {
            cb.accept(piece);
        }
    }

    /** Clears all callbacks. */
    public void clearCallbacks() {
        stateChangeCallbacks.clear();
        scoredCallbacks.clear();
        spawnedCallbacks.clear();
    }

    /** Clears legacy pose sources. */
    public void clear() {
        poseSources.clear();
    }
}

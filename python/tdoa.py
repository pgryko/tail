#!/usr/bin/python3
#
# tdoa.py - Time Difference of Arrival (TDOA) Multilateration Library for Tail
#
# This module provides functions for estimating a position based on range differences
# (derived from TDOA measurements) to a set of reference points (anchors) with known coordinates.
# It implements iterative algorithms based on hyperbolic equations.
#
# Note: The 'ranges' input to these functions likely represents range *differences*
# relative to a reference anchor (B0), not absolute ranges.
#

import math
import numpy as np
import numpy.linalg as lin

from dwarf import * # Imports constants like LIGHT_SPEED, DW1000_CLOCK etc. (though not directly used here)
from numpy import dot # Explicitly import dot product for clarity

# ---------------------------------------------------------------------------
# Helper Functions (Vector Operations)
# ---------------------------------------------------------------------------

def dsq(a):
    """Calculates the squared Euclidean norm (dot product with itself) for each column vector in a."""
    return np.sum((a*a).T,0)

def norm(a):
    """Calculates the Euclidean norm (magnitude) of a vector."""
    return np.sqrt(dsq(a))

def dist(x,y):
    """Calculates the Euclidean distance between two points x and y."""
    return norm(x-y)

# ---------------------------------------------------------------------------
# Core Multilateration Functions
# ---------------------------------------------------------------------------

def hypercone(b0,bi,di):
    """
    Initial position estimate using a hyperbolic intersection approach (non-iterative).
    Solves a system of linear equations derived from the hyperbolic range difference equations.
    This provides a starting point for the iterative `hyperjump` methods.

    Args:
        b0 (np.array): Coordinates of the reference anchor.
        bi (np.array): Coordinates of the other anchors (shape: num_anchors x dimensions).
        di (np.array): Range differences relative to the reference anchor b0 (shape: num_anchors).

    Returns:
        np.array: Initial position estimate (excluding the range difference dimension).
    """
    dim = len(b0)
    bi0 = bi - b0
    di0 = di.reshape(-1,1)
    Gb = np.block([bi0,di0])
    hb = (dsq(bi)-dsq(b0)-di*di)/2
    Gbb = dot(Gb.T,Gb)
    Gbh = dot(Gb.T,hb)
    X = lin.solve(Gbb,Gbh)
    return X[0:dim]

def hyperjump2D(b0,bs,bi,di,sigma,theta):
    """
    Performs one iteration of the 2D hyperbolic multilateration algorithm (Fang algorithm variant?).
    Uses a weighted least-squares approach to refine the position estimate.

    Args:
        b0 (np.array): Coordinates of the reference anchor (2D).
        bs (np.array): Current position estimate (2D).
        bi (np.array): Coordinates of the other anchors (shape: num_anchors x 2).
        di (np.array): Range differences relative to b0 (shape: num_anchors).
        sigma (np.array): Estimated standard deviation for each range difference measurement.
        theta (float): Angular uncertainty parameter (related to clock drift/sync error?).

    Returns:
        tuple: (new_position_estimate (np.array), condition_number (float))
    """
    bi0 = bi - b0
    bs0 = bs - b0
    ds0 = norm(bs0)
    dis = norm(bi - bs)
    di0 = di.reshape(-1,1)
    Gb = np.block([[bi0,di0],[bs0,-ds0],[bs[1],-bs[0],0]])
    hb = np.block([(dsq(bi)-dsq(b0)-di*di)/2, dot(bs0.T,b0), 0])
    Cv = ds0*theta
    Cc = ds0*theta*theta/2
    Pm = dis*sigma
    Ps = np.block([1/Pm,1/Cc,1/Cv])
    Gs = np.diag(Ps*Ps)
    Gbb = dot(dot(Gb.T,Gs),Gb)
    Gbh = dot(dot(Gb.T,Gs),hb)
    X = lin.solve(Gbb,Gbh)
    C = lin.cond(Gbb)
    return X[0:2],C

def hyperlater2D(ref_coord,coords,ranges,sigmas,delta=None,theta=0.045,maxiter=8):
    """
    Iteratively refines a 2D position estimate using `hyperjump2D`.
    Starts with an initial estimate from `hypercone` and iterates until convergence
    (change in position < delta) or max iterations are reached.

    Args:
        ref_coord (list or np.array): Coordinates of the reference anchor (2D).
        coords (list or np.array): Coordinates of the other anchors (list of lists/tuples or Nx2 array).
        ranges (list or np.array): Range differences relative to the reference anchor.
        sigmas (list or np.array): Estimated standard deviation for each range difference.
        delta (float, optional): Convergence threshold (minimum distance change). Defaults to min(sigmas)/2.
        theta (float, optional): Angular uncertainty parameter passed to hyperjump2D. Defaults to 0.045.
        maxiter (int, optional): Maximum number of iterations. Defaults to 8.

    Returns:
        tuple: (final_position_estimate (np.array, 3D with Z=0), final_condition_number (float))

    Raises:
        np.linalg.LinAlgError: If fewer than 3 anchors (excluding reference) are provided.
        ValueError: If ref_coord is not 2D.
    """
    if len(ref_coord) != 2:
        raise ValueError('hyperlater2D only accepts 2D coordinsates')
    if len(coords) < 3:
        raise np.linalg.LinAlgError('Not enough inputs: {}'.format(len(coords)))
    B0 = np.array(ref_coord)
    B = np.array(coords)
    R = np.array(ranges)
    S = np.array(sigmas)
    X = hypercone(B0,B,R)
    Y,C = hyperjump2D(B0,X,B,R,S,theta)
    if delta is None:
        delta = np.amin(S) / 2
    N = 1
    while N < maxiter and dist(X,Y) > delta:
        X = Y
        N = N + 1
        Y,C = hyperjump2D(B0,X,B,R,S,theta)
    X = np.array((Y[0],Y[1],0))
    return X,C

def hyperjump3D(b0,bs,bi,di,sigma,theta):
    """
    Performs one iteration of the 3D hyperbolic multilateration algorithm.
    Similar to hyperjump2D but operates in 3 dimensions.

    Args:
        b0 (np.array): Coordinates of the reference anchor (3D).
        bs (np.array): Current position estimate (3D).
        bi (np.array): Coordinates of the other anchors (shape: num_anchors x 3).
        di (np.array): Range differences relative to b0 (shape: num_anchors).
        sigma (np.array): Estimated standard deviation for each range difference measurement.
        theta (float): Angular uncertainty parameter.

    Returns:
        tuple: (new_position_estimate (np.array), condition_number (float))
    """
    bi0 = bi - b0
    bs0 = bs - b0
    ds0 = norm(bs0)
    dis = norm(bi - bs)
    di0 = di.reshape(-1,1)
    Gb = np.block([[bi0,di0],[bs0,-ds0],[bs[1],-bs[0],0,0],[bs[2],0,-bs[0],0]])
    hb = np.block([(dsq(bi)-dsq(b0)-di*di)/2, dot(bs0.T,b0), 0, 0])
    Cv = ds0*theta
    Cc = ds0*theta*theta/2
    Pm = dis*sigma
    Ps = np.block([1/Pm,1/Cc,1/Cv,1/Cv])
    Gs = np.diag(Ps*Ps)
    Gbb = dot(dot(Gb.T,Gs),Gb)
    Gbh = dot(dot(Gb.T,Gs),hb)
    X = lin.solve(Gbb,Gbh)
    C = lin.cond(Gbb)
    return X[0:3],C

def hyperlater3D(ref_coord,coords,ranges,sigmas,delta=None,theta=0.045,maxiter=8):
    """
    Iteratively refines a 3D position estimate using `hyperjump3D`.
    Starts with an initial estimate from `hypercone` and iterates until convergence
    or max iterations are reached.

    Args:
        ref_coord (list or np.array): Coordinates of the reference anchor (3D).
        coords (list or np.array): Coordinates of the other anchors (list of lists/tuples or Nx3 array).
        ranges (list or np.array): Range differences relative to the reference anchor.
        sigmas (list or np.array): Estimated standard deviation for each range difference.
        delta (float, optional): Convergence threshold. Defaults to min(sigmas)/2.
        theta (float, optional): Angular uncertainty parameter passed to hyperjump3D. Defaults to 0.045.
        maxiter (int, optional): Maximum number of iterations. Defaults to 8.

    Returns:
        tuple: (final_position_estimate (np.array, 3D), final_condition_number (float))

    Raises:
        np.linalg.LinAlgError: If fewer than 4 anchors (excluding reference) are provided.
        ValueError: If ref_coord is not 3D.
    """
    if len(ref_coord) != 3:
        raise ValueError('hyperlater3D only accepts 3D coordinsates')
    if len(coords) < 4:
        raise np.linalg.LinAlgError('Not enough inputs: {}'.format(len(coords)))
    B0 = np.array(ref_coord)
    B = np.array(coords)
    R = np.array(ranges)
    S = np.array(sigmas)
    X = hypercone(B0,B,R)
    Y,C = hyperjump3D(B0,X,B,R,S,theta)
    if delta is None:
        delta = np.amin(S) / 2
    N = 1
    while N < maxiter and dist(X,Y) > delta:
        X = Y
        N = N + 1
        Y,C = hyperjump3D(B0,X,B,R,S,theta)
    return Y,C

# ---------------------------------------------------------------------------
# Pseudo-3D Multilateration (Assumes known Z)
# ---------------------------------------------------------------------------
# These functions adapt the 3D algorithms for scenarios where the Z coordinate
# of the target is assumed to be known or fixed (e.g., operating on a plane).

def hyperjump3Dp(b0,bs,bi,di,sigma,theta):
    """
    Performs one iteration of a pseudo-3D hyperbolic multilateration, solving for X and Y
    while assuming the Z coordinate of the estimate (bs[2]) is fixed.

    Args:
        b0 (np.array): Coordinates of the reference anchor (3D).
        bs (np.array): Current position estimate (3D), bs[2] is used as the fixed Z.
        bi (np.array): Coordinates of the other anchors (shape: num_anchors x 3).
        di (np.array): Range differences relative to b0 (shape: num_anchors).
        sigma (np.array): Estimated standard deviation for each range difference measurement.
        theta (float): Angular uncertainty parameter.

    Returns:
        tuple: (new_position_estimate (np.array, 3D with Z=bs[2]), condition_number (float))
    """
    bi_xy = bi[:,0:2]
    bi_z = bi[:,2]
    b0_xy = b0[0:2]
    b0_z = b0[2]
    bs_xy = bs[0:2]
    bs_z = bs[2]
    bi0_xy = bi_xy - b0_xy
    bi0_z = bi_z - b0_z 
    bs0 = bs - b0
    bs0_xy = bs_xy - b0_xy
    # ci0_z = bi_z*bi_z - b0_z*b0_z -2*bs_z*bi0_z
    #       = (bi_z - b0_z)(bi_z + b0_z) - 2bs_z*bi0_z
    #       = (bi_z - b0_z)(bi_z + b0_z - 2bs_z)
    #       = (bi_z - b0_z)((bi_z - bs_z) + (b0_z - bs_z))
    ci0_z = bi0_z * ((bi_z - bs_z) + (b0_z - bs_z))
    ds0 = norm(bs0)
    dis = norm(bi - bs)
    di0 = di.reshape(-1,1)
    Gb = np.block([[bi0_xy,di0],[bs0_xy,-ds0],[bs[1],-bs[0],0]])
    hb = np.block([(dsq(bi_xy)-dsq(b0_xy)-di*di+ci0_z)/2, dot(bs0_xy.T,b0_xy), 0])
    Cv = ds0*theta
    Cc = ds0*theta*theta/2
    Pm = dis*sigma
    Ps = np.block([1/Pm,1/Cc,1/Cv])
    Gs = np.diag(Ps*Ps)
    Gbb = dot(dot(Gb.T,Gs),Gb)
    Gbh = dot(dot(Gb.T,Gs),hb)
    X = lin.solve(Gbb,Gbh)
    C = lin.cond(Gbb)
    R = np.array((X[0],X[1],bs[2]))
    return R,C

def hyperlater3Dp(ref_coord,coords,ranges,sigmas,delta=None,theta=0.045,maxiter=8,z_est=0.0):
    """
    Iteratively refines a pseudo-3D position estimate using `hyperjump3Dp`.
    Starts with an initial 2D estimate from `hypercone` (using XY coords) combined
    with the provided `z_est`, then iterates until convergence or max iterations.

    Args:
        ref_coord (list or np.array): Coordinates of the reference anchor (3D).
        coords (list or np.array): Coordinates of the other anchors (list of lists/tuples or Nx3 array).
        ranges (list or np.array): Range differences relative to the reference anchor.
        sigmas (list or np.array): Estimated standard deviation for each range difference.
        delta (float, optional): Convergence threshold. Defaults to min(sigmas)/2.
        theta (float, optional): Angular uncertainty parameter passed to hyperjump3Dp. Defaults to 0.045.
        maxiter (int, optional): Maximum number of iterations. Defaults to 8.
        z_est (float, optional): The assumed fixed Z coordinate for the target. Defaults to 0.0.

    Returns:
        tuple: (final_position_estimate (np.array, 3D), final_condition_number (float))

    Raises:
        np.linalg.LinAlgError: If fewer than 4 anchors (excluding reference) are provided.
        ValueError: If ref_coord is not 3D.
    """
    if len(ref_coord) != 3:
        raise ValueError('hyperlater_pseudo3D only accepts 3D coordinsates')
    if len(coords) < 4:
        raise np.linalg.LinAlgError('Not enough inputs: {}'.format(len(coords)))
    B0 = np.array(ref_coord)
    B = np.array(coords)
    R = np.array(ranges)
    S = np.array(sigmas)
    X = hypercone(B0[0:2],B[:,0:2],R)
    X = np.array((X[0],X[1],z_est))
    Y,C = hyperjump3Dp(B0,X,B,R,S,theta)
    if delta is None:
        delta = np.amin(S) / 2
    N = 1
    while N < maxiter and dist(X,Y) > delta:
        X = Y
        N = N + 1
        Y,C = hyperjump3Dp(B0,X,B,R,S,theta)
    return Y,C

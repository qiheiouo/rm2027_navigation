"""Independent vector planar geometry; finite inputs checked by evidence readers."""
import numpy as np


def rotate(poses,points):
    poses=np.asarray(poses,dtype=float);points=np.asarray(points,dtype=float)
    c=np.cos(poses[...,2])[...,None];s=np.sin(poses[...,2])[...,None]
    return np.stack((poses[...,0,None]+c*points[:,0]-s*points[:,1],
                     poses[...,1,None]+s*points[:,0]+c*points[:,1]),axis=-1)


def polygon_distance(a,b):
    """Convex polygon separation plus symmetric point-to-edge distance."""
    a=np.asarray(a,dtype=float);b=np.asarray(b,dtype=float)
    shape=np.broadcast_shapes(a.shape[:-2],b.shape[:-2])
    a=np.broadcast_to(a,shape+a.shape[-2:]);b=np.broadcast_to(b,shape+b.shape[-2:])
    ea=np.roll(a,-1,axis=-2)-a;eb=np.roll(b,-1,axis=-2)-b
    edges=np.concatenate((ea,eb),axis=-2);axes=np.stack((-edges[...,1],edges[...,0]),axis=-1)
    pa=np.einsum('...vi,...ei->...ev',a,axes);pb=np.einsum('...vi,...ei->...ev',b,axes)
    separate=np.any((pa.max(axis=-1)<pb.min(axis=-1))|(pb.max(axis=-1)<pa.min(axis=-1)),axis=-1)
    def nearest(vertices,edges,points):
        delta=points[...,None,:,:]-vertices[...,:,None,:]
        denominator=np.sum(edges*edges,axis=-1)[...,None]
        numerator=np.sum(delta*edges[...,:,None,:],axis=-1)
        fraction=np.divide(numerator,denominator,out=np.zeros_like(numerator),where=denominator>0)
        delta=delta-np.clip(fraction,0,1)[...,None]*edges[...,:,None,:]
        return np.sum(delta*delta,axis=-1).min(axis=(-1,-2))
    squared=np.minimum(nearest(a,ea,b),nearest(b,eb,a))
    return np.where(separate,np.sqrt(squared),0.)


def circle_box_gap(centers,radius,box_poses,dimensions):
    delta=np.asarray(centers)-np.asarray(box_poses)[...,:2]
    c=np.cos(box_poses[...,2]);s=np.sin(box_poses[...,2])
    local=np.stack((c*delta[...,0]+s*delta[...,1],-s*delta[...,0]+c*delta[...,1]),axis=-1)
    outside=np.maximum(np.abs(local)-np.asarray(dimensions)/2,0)
    return np.maximum(np.sqrt(np.sum(outside*outside,axis=-1))-radius,0.)


def travel(a,b,radius):
    delta=b-a;angle=np.remainder(delta[...,2]+np.pi,2*np.pi)-np.pi
    return np.linalg.norm(delta[...,:2],axis=-1)+radius*np.abs(angle)


def interpolation_lower(gaps,robot_poses,robot_radius,obstacle_poses=None,obstacle_radius=0):
    motion=travel(robot_poses[...,:-1,:],robot_poses[...,1:,:],robot_radius)
    if obstacle_poses is not None:motion=motion+travel(obstacle_poses[:-1],obstacle_poses[1:],obstacle_radius)
    lower=np.minimum(gaps[...,:-1],gaps[...,1:])-.5*motion
    return np.minimum(gaps.min(axis=-1),lower.min(axis=-1))


def raw_clearance_lower(poses,footprint,record,reserves,threshold=203):
    """Conservative distance to blocked cells/outside; distant pairs use disk bound.

    Only exact polygon distance can reject a near pair. The disk lower bound
    certifies far pairs when greater than that pose's interval reserve.
    """
    shape=poses.shape[:-1];poses=np.asarray(poses).reshape(-1,3);reserve=np.asarray(reserves).reshape(-1)
    poly=rotate(poses,footprint);res=record['resolution'];width,height=record['size'];origin=np.asarray(record['origin'])
    values=(np.frombuffer(record['data'],dtype=np.uint8) if isinstance(record['data'],bytes)
            else np.asarray(record['data'],dtype=np.uint8))
    grid=values.reshape(height,width)
    ys,xs=np.nonzero(grid>=threshold)
    lo=poly.min(axis=1);hi=poly.max(axis=1)
    boundary=np.minimum(np.min(lo-origin,axis=1),np.min(origin+np.array([width,height])*res-hi,axis=1))
    result=np.maximum(boundary,0.)
    if not len(xs):return result.reshape(shape)
    cells=origin+np.column_stack((xs,ys))*res
    centers=cells+res/2;radius=np.linalg.norm(np.asarray(footprint),axis=1).max();cell_radius=res/np.sqrt(2)
    corners=np.array([[0,0],[res,0],[res,res],[0,res]])
    for start in range(0,len(poses),512):
        end=min(start+512,len(poses))
        circle_bound=np.linalg.norm(poses[start:end,None,:2]-centers[None,:,:],axis=-1)-radius-cell_radius
        near=circle_bound<=reserve[start:end,None]+1e-9
        ii,jj=np.nonzero(near)
        if len(ii):circle_bound[ii,jj]=polygon_distance(poly[start:end][ii],cells[jj,None,:]+corners)
        result[start:end]=np.minimum(result[start:end],np.maximum(circle_bound.min(axis=1),0.))
    return result.reshape(shape)

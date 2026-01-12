function posvel = gcPosVel( model, xfb, q, qd )
import casadi.*
posvel = SX.zeros(6, length(model.gc.body));

% gcPosVel  calculate positions and velocities of contact points
% gcPosVel(model,q,qd), gcPosVel(model,xfb,q,qd) and gcPosVel(model,xfb)
% calculate a 4xn or 6xn matrix, depending on whether the model is planar
% or spatial, containing the position and linear velocity of every point
% specified in model.gc.point (n==length(model.gc.point)).  The position
% coordinates appear in the top rows, and the velocity coordinates at the
% bottom.  All are expressed in base (i.e., absolute) coordinates.  The
% data for model.gc.point(i) appears in column i.  The argument are: the
% model data structure (which must contain a .gc substructure); the
% position and velocity variables of a floating base (as defined by FDfb
% and IDfb); the joint position variables; and the joint velocity
% variables.  The call gcPosVel(model,xfb) is permissible only if the
% model describes a single floating rigid body.

if nargin==4 || nargin==2		% xfb supplied

  qn = xfb(1:4);			% unit quaternion fixed-->f.b.
  r = xfb(5:7);				% position of f.b. origin
  Xa{6} = plux( rq(qn), r );		% xform fixed --> f.b. coords

  vfb = xfb(8:end);
  vb{6} = Xa{6} * vfb;			% f.b. vel in f.b. coords

  for i = 7:model.NB
    [ XJ, S ] = jcalc( model.jtype{i}, q(i-6) );
    Xup = XJ * model.Xtree{i};
    vJ = S*qd(i-6);
    Xa{i} = Xup * Xa{model.parent(i)};
    vb{i} = Xup * vb{model.parent(i)} + vJ;
  end

else					% xfb not supplied

  qd = q;  q = xfb;			% shift up the arguments

  for i = 1:model.NB
    [ XJ, S ] = jcalc( model.jtype{i}, q(i) );
    Xup = XJ * model.Xtree{i};
    vJ = S*qd(i);
    if model.parent(i) == 0
      Xa{i} = Xup;
      vb{i} = vJ;
    else
      Xa{i} = Xup * Xa{model.parent(i)};
      vb{i} = Xup * vb{model.parent(i)} + vJ;
    end
  end

end
counter = 1; 
for i = unique(model.gc.body)
  X = inv(Xa{i});			% xform body i -> abs coords
  v = X * vb{i};			% body i vel in abs coords
  iset = model.gc.body == i;		% set of points assoc with body i
  pt = Xpt( X, model.gc.point(:,iset) );	% xform points to abs coords
  vpt = Vpt( v, pt );			% linear velocities of points
  posvel(:,counter) = [ pt; vpt ];		% insert into correct columns
  counter = counter + 1; 
end

% These functions need to be added here again for it to work with casadi
function  xp = Xpt( X, p )
import casadi.*
% Xpt  apply Plucker/planar coordinate transform to 2D/3D points
% xp=Xpt(X,p)  applies the coordinate transform X to the points in p,
% returning the new coordinates in xp.  If X is a 6x6 matrix then it is
% taken to be a Plucker coordinate transform, and p is expected to be a
% 3xn matrix of 3D points.  Otherwise, X is assumed to be a planar
% coordinate transform and p a 2xn array of 2D points.

if all(size(X)==[6 6])			% 3D points
  E = X(1:3,1:3);
  r = -skew(E'*X(4:6,1:3));
else					% 2D points
  E = X(2:3,2:3);
  r = [ X(2,3)*X(2,1)+X(3,3)*X(3,1); X(2,3)*X(3,1)-X(3,3)*X(2,1) ];
end

if size(p,2) > 1
  r = repmat(r,1,size(p,2));
end

xp = E * (p - r);
end

function  out = skew( in )
import casadi.*
% skew  convert 3D vector <--> 3x3 skew-symmetric matrix
% S=skew(v) and v=skew(A) calculate the 3x3 skew-symmetric matrix S
% corresponding to the given 3D vector v, and the 3D vector corresponding
% to the skew-symmetric component of the given arbitrary 3x3 matrix A.  For
% vectors a and b, skew(a)*b is the cross product of a and b.  If the
% argument is a 3x3 matrix then it is assumed to be A, otherwise it is
% assumed to be v.  skew(A) produces a column-vector result, but skew(v)
% will accept a row or column vector argument.

if all(size(in)==[3 3])			% do v = skew(A)
  out = 0.5 * [ in(3,2) - in(2,3);
		in(1,3) - in(3,1);
		in(2,1) - in(1,2) ];
else					% do S = skew(v)
  out = [  0,    -in(3),  in(2);
	   in(3),  0,    -in(1);
	  -in(2),  in(1),  0 ];
end
end

end

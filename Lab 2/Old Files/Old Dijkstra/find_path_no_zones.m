% Load the original map
map_original = imread('bw.png');

% Resize
map_im = imresize(map_original, 0.5);
height = size(map_im, 1);
width = size(map_im, 2);

% Zeros are the places where the robot can move
n_zeros = sum(map_im(:) == 0);
n_ones = sum(map_im(:) == 255);

% Matrix with the correspondence between pixel and node number
pixel_number = zeros(height, width);
% Correspondence between pixel number and indices in the image
indices = zeros(n_zeros, 2);

% Pixels where the robot cannot go have node number -1
count = 1;
for i=1:height
    for j=1:width
        if(map_im(i,j) == 0)
            pixel_number(i,j) = count;
            indices(count,1) = i;
            indices(count,2) = j;
            count = count + 1;
        else
            pixel_number(i,j) = -1;
        end
    end
end

% Adjacency matrix - all weights are 0 if nodes are not connected and 1 if
% they are connected
G = zeros(n_zeros, n_zeros);
    
for i=1:height
    for j=1:width
        if map_im(i,j) == 0
            if i > 1 && map_im(i-1,j) == 0
                G(pixel_number(i,j), pixel_number(i-1,j)) = 1;
            end
            if i < height && map_im(i+1,j) == 0
                G(pixel_number(i,j), pixel_number(i+1,j)) = 1;
            end
            if j > 1 && map_im(i,j-1) == 0
                G(pixel_number(i,j), pixel_number(i,j-1)) = 1;
            end
            if j < width && map_im(i,j+1) == 0
                G(pixel_number(i,j), pixel_number(i,j+1)) = 1;
            end
        end
    end
end

% imshow(map_im);
% start_pixel = ginput(1);
% end_pixel = ginput(1);
% 
% close all

% start_pixel = pixel_number(round(start_pixel(1)),round(start_pixel(2)))
% end_pixel = pixel_number(round(end_pixel(1)),round(end_pixel(2)))

% Predefined initial and finish positions in node number
start_pixel = pixel_number(104, 128);
end_pixel = pixel_number(373, 418);

% Dijkstra
[dist, paths] = dijk(G,start_pixel,end_pixel);

% Color the original map with the path in red
map_rgb_path = uint8(map_original(:,:,[1 1 1])*255);
for i=1:size(paths,1)-1
    r = 2*indices(paths(i,1),1);
    c = 2*indices(paths(i,1),2);
    r_2 = 2*indices(paths(i+1,1),1);
    c_2 = 2*indices(paths(i+1,1),2);
    map_rgb_path(r,c,1) = 255;
    map_rgb_path(r,c,2) = 0;
    map_rgb_path(r,c,3) = 0;
    map_rgb_path(round(r+(r_2-r)/2),round(c+(c_2-c)/2),1) = 255;
    map_rgb_path(round(r+(r_2-r)/2),round(c+(c_2-c)/2),2) = 0;
    map_rgb_path(round(r+(r_2-r)/2),round(c+(c_2-c)/2),3) = 0;
end
r = 2*indices(paths(size(paths,1),1),1);
c = 2*indices(paths(size(paths,1),1),2);
map_rgb_path(r,c,1) = 0;
map_rgb_path(r,c,2) = 0;
map_rgb_path(r,c,3) = 0;

imshow(map_rgb_path)

%% Auxiliary Functions

function [dist, varargout] = dijk(W, start_verts, end_verts)

[dist, P] = dijk_(W, start_verts, end_verts);
if nargout==2
    varargout{1} = pred2path_(P, start_verts, end_verts);
end

end

function [D, P] = dijk_(A,s,t)

[n,cA] = size(A);

if nargin < 2 || isempty(s), s = (1:n)'; else s = s(:); end
if nargin < 3 || isempty(t), t = (1:n)'; else t = t(:); end

if ~any(any(tril(A) ~= 0))
   isAcyclic = 1;
elseif ~any(any(triu(A) ~= 0))
   isAcyclic = 2;
else
   isAcyclic = 0;
end

if n ~= cA
   error('A must be a square matrix');
elseif ~isAcyclic && any(any(A < 0))
   error('A must be non-negative');
elseif any(s < 1 | s > n)
   error(['''s'' must be an integer between 1 and ',num2str(n)]);
elseif any(t < 1 | t > n)
   error(['''t'' must be an integer between 1 and ',num2str(n)]);
end

A = A';

D = zeros(length(s),length(t));
P = zeros(length(s),n);

for i = 1:length(s)
   j = s(i);

   Di = Inf*ones(n,1); Di(j) = 0;

   isLab = false(length(t),1);
   if isAcyclic ==  1
      nLab = j - 1;
   elseif isAcyclic == 2
      nLab = n - j;
   else
      nLab = 0;
      UnLab = 1:n;
      isUnLab = true(n,1);
   end

   while nLab < n && ~all(isLab)
      if isAcyclic
         Dj = Di(j);
      else
         [Dj,jj] = min(Di(isUnLab));
         j = UnLab(jj);
         UnLab(jj) = [];
         isUnLab(j) = 0;
      end

      nLab = nLab + 1;
      if length(t) < n, isLab = isLab | (j == t); end

      [jA,~,Aj] = find(A(:,j));
      Aj(isnan(Aj)) = 0;

      if isempty(Aj), Dk = Inf; else Dk = Dj + Aj; end

      P(i,jA(Dk < Di(jA))) = j;
      Di(jA) = min(Di(jA),Dk);

      if isAcyclic == 1
         j = j + 1;
      elseif isAcyclic == 2
         j = j - 1;
      end
      
   end
   D(i,:) = Di(t)';
end

end

function rte = pred2path_(P,s,t)

[~,n] = size(P);

if nargin < 2 || isempty(s), s = (1:n)'; else s = s(:); end
if nargin < 3 || isempty(t), t = (1:n)'; else t = t(:); end

if any(P < 0 | P > n)
   error(['Elements of P must be integers between 1 and ',num2str(n)]);
elseif any(s < 1 | s > n)
   error(['"s" must be an integer between 1 and ',num2str(n)]);
elseif any(t < 1 | t > n)
   error(['"t" must be an integer between 1 and ',num2str(n)]);
end

rte = cell(length(s),length(t));

[~,idxs] = find(P==0);

for i = 1:length(s)
    
   si = find(idxs == s(i));
   for j = 1:length(t)
      tj = t(j);
      if tj == s(i)
         r = tj;
      elseif P(si,tj) == 0
         r = [];
      else
         r = tj;
         while tj ~= 0
            if tj < 1 || tj > n
               error('Invalid element of P matrix found.')
            end
            r = [P(si,tj); r];
            tj = P(si,tj);
         end
         r(1) = [];
      end
      rte{i,j} = r;
   end
end

if length(s) == 1 && length(t) == 1
   rte = rte{:};
end

while 0
   if t < 1 || t > n || round(t) ~= t
      error('Invalid "pred" element found prior to reaching "s"');
   end
   rte = [P(t) rte];
   t = P(t);
end
end
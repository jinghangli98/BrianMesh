clear; clc;
cd('/Users/jinghangli/Desktop/Pitt Summer 2020/AI_computation/ImgAI')
%% Plot settings
fontSize=15;
faceAlpha1=0.3;
faceAlpha2=1;
cMap=gjet(4); 
patchColor=cMap(1,:);
markerSize=25; 
%%
importdata('data.mat');
hdr = ans.hdr;
matrix = hdr(1).img;
[A,B,C] = size(matrix);
% [node,elem,face]=v2m(abs(matrix)>0,0.2,0.2,30,'simplify');
% N = node; %whole brain
% conn = meshconn(face(:,1:3), size(N,1));
% N=smoothsurf(N,[],conn,10,0.5,'laplacian');
% 
% plotmesh(N,face,elem,'x>40','linestyle',':')
v_pos = [];
v_neg = [];
V=[];
counter_pos=1;
counter_neg=1;
counter =1;
for i = 1:A
    for j = 1:B
        for k = 1:C
if isnan(matrix(i,j,k)) == 0
V(counter,:) = [i,j,k]; 
%colormap(counter,1) = floor(matrix(i,j,k));
counter = counter +1;

end
        end
    end
end

for i = 1:A
    for j = 1:B
        for k = 1:C
if isnan(matrix(i,j,k)) == 0 && matrix(i,j,k) > 0
v_pos(counter_pos,:) = [i,j,k]; 
counter_pos = counter_pos +1;
elseif isnan(matrix(i,j,k)) == 0 && matrix(i,j,k) < 0
    v_neg(counter_neg,:) = [i,j,k]; 
counter_neg = counter_neg +1;
end
        end
    end
end

x = V(:,1);
y = V(:,2);
z = V(:,3);
V2 = [v_neg(:,1),v_neg(:,2),v_neg(:,3)];
V3 = [v_pos(:,1),v_pos(:,2),v_pos(:,3)];
shp = alphaShape(x,y,z);
shp1 = alphaShape(v_neg(:,1),v_neg(:,2),v_neg(:,3));
shp2 = alphaShape(v_pos(:,1),v_pos(:,2),v_pos(:,3));
F = boundaryFacets(shp);
F2 = boundaryFacets(shp1);
F3 = boundaryFacets(shp2);
%%
C=ones(size(F,1),1); %Face boundary markers (aka face colors)
V_regions=getInnerPoint(F,V); %Define region points
V_holes=[]; %Define hole points
[regionTetVolumes]=tetVolMeanEst(F,V); %Volume estimate for regular tets
stringOpt='-pq1.2AaY'; %Options for tetgen
%%
% Mesh using TetGen

%Create tetgen input structure
inputStruct.stringOpt=stringOpt; %Tetgen options
inputStruct.Faces=F; %Boundary faces
inputStruct.Nodes=V; %Nodes of boundary
inputStruct.faceBoundaryMarker=C; 
inputStruct.regionPoints=V_regions; %Interior points for regions
inputStruct.holePoints=V_holes; %Interior points for holes
inputStruct.regionA=regionTetVolumes; %Desired tetrahedral volume for each region

% Mesh model using tetrahedral elements using tetGen 
[meshOutput]=runTetGen(inputStruct); %Run tetGen 
meshOutput.boundaryMarker = floor(10*rand(size(C)));

%% 
% Access mesh output structure
E=meshOutput.elements; %The elements
V=meshOutput.nodes; %The vertices or nodes
CE=meshOutput.elementMaterialID; %Element material or region id
Fb=meshOutput.facesBoundary; %The boundary faces
Cb=meshOutput.boundaryMarker; %The boundary markers

%% Activation asymmetry index method
%smoothing
N = V; %whole brain
N3 = V3; %activated region
conn = meshconn(F(:,1:3), size(N,1));
conn3 = meshconn(F3(:,1:3), size(N3,1));
N=smoothsurf(N,[],conn,10,0.5,'laplacian');
N3=smoothsurf(N3,[],conn3,10,0.5,'laplacian');

%Graphing
gpatch(F,N,zeros(size(F,1),1),'k',faceAlpha2)
axisGeom(gca,fontSize); 
hold on 
gpatch(F3,N3,ones(size(F3,1),1),'r',faceAlpha1)
gdrawnow;
hold off
colormap(cMap); icolorbar;
legend({'Resting Region','Activated Region'},'Location','NorthWestOutside');
axisGeom(gca,fontSize); 
%% Activation FWHM 
%smoothing
N = V; %whole brain
N3 = V3; %activated region
conn = meshconn(F(:,1:3), size(N,1));
conn3 = meshconn(F3(:,1:3), size(N3,1));
N=smoothsurf(N,[],conn,10,0.5,'laplacian');
N3=smoothsurf(N3,[],conn3,10,0.5,'laplacian');

% Mesh using TetGen
stringOpt='-pq2/0';%'-pq1.2AaY'; %Options for tetgen
%Create tetgen input structure
inputStruct.stringOpt=stringOpt; %Tetgen options
inputStruct.Faces=F; %Whole Brain faces
inputStruct.Nodes=N; %Whole Brain nodes
inputStruct.faceBoundaryMarker=C; 
inputStruct.regionPoints=V_regions; %Interior points for regions. For the whole brain
inputStruct.holePoints=V_holes; %Interior points for holes
inputStruct.regionA=regionTetVolumes; %Desired tetrahedral volume for each region

% Mesh model using tetrahedral elements using tetGen 
[meshOutput]=runTetGen(inputStruct); %Run tetGen 

node = meshOutput.nodes;
face = meshOutput.faces;
element = meshOutput.elements;
gpatch(face,node)
colormap(cMap); icolorbar;
axisGeom(gca,fontSize); 

%RE-MESH





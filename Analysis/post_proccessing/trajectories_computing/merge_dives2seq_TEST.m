function    seq_out = merge_seq(dives)
%
%     s = merge_fields(s1,s2)
%     Merge the fields of minimum two dives sctructure. If there are duplicate
%     fields, the fields in s1 are taken.
%
%		Inputs:
%		dives sctructure containing at least 2 dives.
%
%		Returns:
%		s is a structure containing all of the fields in s1 and s2.
%
%		Examples:
%		s1 = struct('a',1,'b',[2 3 4])
%		s2 = struct('b',3,'c','cat')
%		s = merge_fields(s1,s2)
%		Returns: s containing s.b=[2,3,4], s.c='cat', s.a=1
%
%     TODO : Auto merge
%
%     Valid: Matlab, Octave
%     markjohnson@st-andrews.ac.uk
%     Last modified: 5 May 2017

if length(dives(:,1)) < 2
    fprintf('merge_dives : Need at least 2 dives\n') ;
    help merge_dives
    return
end

if ~isstruct(dives(:,1))
    fprintf('merge_dives : Dives need to be sctruct \n') ;
    return
end

seq_out = dives(1,1);
seq_out.tend = dives(end,1).tend;
seq_out.type = 'seq';

Z = fieldnames(dives(1,1).var);


len = length(dives(:,1));

for i=2:len   
    
    seq_out.var.P.data = [seq_out.var.P.data; dives(i,1).var.P.data];  
    seq_out.var.LAT.data = [seq_out.var.LAT.data; dives(i,1).var.LAT.data];  
    seq_out.var.LON.data = [seq_out.var.LON.data; dives(i,1).var.LON.data];  
    dives(i,1).var.B.data(:,2:3) = dives(i,1).var.B.data(:,2:3)+dives(i-1,1).var.B.data(end,3);
    %seq_out.var.B.data(:,2:3) = [seq_out.var.B.data(:,2:3);  dives(i,1).var.B.data(:,2:3];
    seq_out.var.B.data = [seq_out.var.B.data; dives(i,1).var.B.data];
    %seq_out.var.B.data(:,2:3) = seq_out.var.B.data(:,2:3)+dives(i-1,1).var.B.data(end,3);    
end

%     
% for k=1:length(Z)
%     s = setfield(seq_out.var,Z{k},getfield(dives(1,1).var,Z{k})) ;
% end
% 
% 
% len = length(dives(1,:));
% for i=2:len
%     
%     Z = fieldnames(dives(i,1).var);
%     
%     for k=1:length(Z)
%         setfield(seq_out.var,Z{k},[ getfield(dives(1,1).var,Z{k})) ;
%         seq_out.var.Z{k} = [seq_out.var.Z{k}(:,:) dives(i,1).var.Z{k}(:,:)] ;
%     end
%     
% end

end

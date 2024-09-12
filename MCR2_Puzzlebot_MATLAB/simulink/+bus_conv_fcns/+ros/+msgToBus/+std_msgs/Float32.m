function slBusOut = Float32(msgIn, slBusOut, varargin)
%#codegen
%   Copyright 2021-2022 The MathWorks, Inc.
    slBusOut.Data = single(msgIn.Data);
end

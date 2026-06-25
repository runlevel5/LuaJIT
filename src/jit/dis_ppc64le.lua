----------------------------------------------------------------------------
-- LuaJIT PPC64LE disassembler wrapper module.
--
-- Copyright (C) 2005-2026 Mike Pall. All rights reserved.
-- Released under the MIT license. See Copyright Notice in luajit.h
----------------------------------------------------------------------------
-- This module just exports the little-endian functions from the
-- PPC disassembler module. All the interesting stuff is there.
------------------------------------------------------------------------------

local dis_ppc = require((string.match(..., ".*%.") or "").."dis_ppc")
return {
  create = dis_ppc.create_el,
  disass = dis_ppc.disass_el,
  regname = dis_ppc.regname
}

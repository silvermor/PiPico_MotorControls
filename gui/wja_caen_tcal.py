#! /usr/bin/env python3

# wja_caen_tcal.py
# begun 2024-02-21 by wja
# starting from bits of wja_rocstar_init.py

# purpose: implement DRS4 timing calibration for CAEN N6742 board,
# using external Rigol DG4162 AWG as source of 100 MHz sine wave


import math
import os
import random
import re
import sys
import time

import numpy as np
import matplotlib.pyplot as plt
import scipy
import scipy.optimize
import scipy.special
import pandas as pd
import h5py

import nbutil

from collections import namedtuple
from dataclasses import dataclass, field
from types import SimpleNamespace
from typing import Any, List

import pyvisa


class WaveReader:

    @classmethod
    def read_next_header_line(cls, fp, info, expect, key, check=True):
        l = fp.readline()
        if l == "":
            raise EOFError()
        info.lines_read = getattr(info, "lines_read", 0) + 1
        l = l.rstrip()
        if not l.startswith(expect):
            raise ValueError(f"expect '{expect}' in '{l}'")
        value = l.split(":")[1].strip()
        if value.startswith("0x"):
            value = int(value[2:], 16)
        elif value == "TR_0_0":
            value = 16
        elif value == "TR_0_1":
            value = 17
        else:
            value = int(value)
        if hasattr(info, key) and check:
            if value != info.__dict__[key]:
                print(f"'{value}' != '{info.__dict__[key]}'")
            assert value == info.__dict__[key]
        info.__dict__[key] = value

    @classmethod
    def wave_read(cls, fnam):
        info = SimpleNamespace()
        info.lines_read = 0
        info.file_name = fnam
        fp = open(fnam)
        events = []
        while 1:
            try:
                cls.read_next_header_line(fp, info,
                                          "Record Length:",
                                          "record_length")
            except EOFError:
                break
            cls.read_next_header_line(fp, info,
                                      "BoardID:",
                                      "board_id")
            cls.read_next_header_line(fp, info,
                                      "Channel:",
                                      "channel",
                                      check=False)
            cls.read_next_header_line(fp, info,
                                      "Event Number:",
                                      "event_number",
                                      check=False)
            cls.read_next_header_line(fp, info,
                                      "Pattern:",
                                      "pattern")
            cls.read_next_header_line(fp, info,
                                      "Trigger Time Stamp:",
                                      "trigger_time_stamp",
                                      check=False)
            cls.read_next_header_line(fp, info,
                                      "DC offset (DAC):",
                                      "offset_dac")
            cls.read_next_header_line(fp, info,
                                      "Start Index Cell:",
                                      "start_cell",
                                      check=False)
            wave = []
            for i in range(info.record_length):
                l = fp.readline().strip()
                info.lines_read += 1
                wave.append(float(l))
            event = SimpleNamespace()
            event.drs = np.array(wave)
            event.evnum = info.event_number
            event.tstamp = info.trigger_time_stamp
            event.drs_trig_cell = info.start_cell
            events.append(event)
        print(f"read {info.lines_read} lines, {len(events)} events "
              f"from {info.file_name}")
        return info, events


@dataclass
class PedestalRun:
    ichannel: int
    events: List[Any]
    peds: Any  # numpy array


@dataclass
class DcOffsetRun:
    ichnl: int
    dc_volts: float
    psum: Any  # numpy array


@dataclass
class TcalInfo:
    fNominalFrequency : float  # [GHz]
    llim : int = field(default=-400)
    ulim : int = field(default=+400)
    ncell : int = field(default=1024)
    fTcalFrequency : float = field(default=0.1)  # [GHz]
    nIterPeriod : int = 10000
    corr_limit : float = 0.001
        
    def analyze_slope(self, obj, calculate=False):
        s = obj
        info = self
        wf = s.wf
        #
        # Stefan's code (DRSBoard::AnalyzeSlope in DRS.cpp) skips +/-
        # 5 cells around the trigger cell, but is not using ROI
        # readout mode.  On the ROCSTAR board we will use ROI
        # readout mode.
        #
        # First look for rising edges
        for isample in range(5, len(wf)-5):
            icell = (s.tcell + isample)%info.ncell
            # Test slope between previous and next cell to allow for
            # negative cell width
            avg = (wf[isample] + wf[isample+1])/2
            if ((wf[isample-1] < avg) and
                (avg < wf[isample+2]) and
                (abs(wf[isample]) < info.ulim) and
                (abs(wf[isample+1]) < info.ulim)):
                # Calculate delta_v
                dv = wf[isample+1] - wf[isample]
                # Accumulate list of delta_v[icell] for mean
                s.dv_list[icell].append(dv)
        # Next look for falling edges
        for isample in range(5, len(wf)-5):
            icell = (s.tcell + isample)%info.ncell
            # Test slope between previous and next cell to allow for
            # negative cell width
            avg = (wf[isample] + wf[isample+1])/2
            if ((wf[isample-1] > avg) and
                (avg > wf[isample+2]) and
                (abs(wf[isample]) < info.ulim) and
                (abs(wf[isample+1]) < info.ulim)):
                # Calculate delta_v
                dv = wf[isample+1] - wf[isample]
                # Accumulate list of delta_v[icell] for mean
                s.dv_list[icell].append(-dv)
        # Calculate calibration (presumably on final call)
        if calculate:
            # average over all 1024 dU
            sum = 0
            s.celldv = np.zeros(info.ncell)
            nmissing = 0
            import matplotlib.pyplot as plt
            for i in range(info.ncell):
                if len(s.dv_list[i])==0:
                    nmissing += 1
                else:
                    s.dv_list[i].sort()
                    l = np.array(s.dv_list[i])
                    # truncated mean
                    lo = len(l)*1//5
                    hi = len(l)*4//5 + 1
                    s.celldv[i] = np.mean(l[lo:hi])
                    if 0:
                        print(f"{i} {len(l)} {l.min():.0f} {l.max():.0f} "
                              f"rms={l[lo:hi].std():.2f} "
                              f"{l.mean():.2f} {np.median(l):.2f} "
                              f"{s.celldv[i]:.2f}")
                        plt.hist(l, bins=50)
                        plt.show()
                sum += s.celldv[i]
            if nmissing:
                print("nmissing={} iiter={} analyze_slope"
                      .format(nmissing, s.iiter))
                for i in range(info.ncell):
                    if len(s.dv_list[i])==0:
                        # replace missing value with avg of odd/even cells
                        s.celldv[i] = s.celldv[i%2::2].mean()
                sum = np.sum(s.celldv)
            sum /= info.ncell
            dtcell = 1.0/info.fNominalFrequency
            for i in range(info.ncell):
                s.celldt[i] = dtcell * s.celldv[i] / sum
            cellnum = np.arange(info.ncell)
            plt.plot(cellnum[0::2], 1000*s.celldt[0::2], '.')
            plt.plot(cellnum[1::2], 1000*s.celldt[1::2], '.')
            plt.grid()
            plt.xlabel("cellwidth [ps] after analyze_slope "
                       "(before analyze_period)")
            plt.show()
            inl = np.cumsum(s.celldt) - s.celldt.mean()*np.arange(info.ncell)
            plt.plot(np.arange(info.ncell)[0::2], inl[0::2], '-')
            plt.grid()
            plt.xlabel("INL [ns] vs cell (even cells only)")
            plt.show()
    
    def analyze_period(self, obj, iiter):
        s = obj
        info = self
        wf = s.wf
        # correct for zero common mode
        wf -= wf.mean()
        self.wf = wf.copy()
        # calculate time axis
        tc = s.tcell
        cw = np.roll(s.celldt, 1-tc)
        tcor = np.cumsum(cw) - cw[0]
        # estimate damping factor
        damping0 = 0.01
        damping = damping0/((math.log(max(2,iiter))/math.log(2)))
        # estimate number of zero crossings
        ns_per_tcal = 1.0/info.fTcalFrequency
        ns_per_sweep = info.ncell/info.fNominalFrequency
        tcal_per_sweep = ns_per_sweep/ns_per_tcal
        cells_per_tcal = info.ncell/tcal_per_sweep
        nest = int(tcal_per_sweep)
        dv_limit = 25 # 100  # 10
        ncomplain = 0
        s.dv_values = []
        # loop over falling edges, then rising edges
        for edge in range(2):
            # find zero crossings
            zero_xing = []
            for isample in range(10,len(wf)-10):
                icell = (s.tcell + isample)%info.ncell
                dv = abs(wf[isample+1] - wf[isample])
                if edge==0:
                    # look for falling edge
                    if ((wf[isample-1] > wf[isample])   and
                        (wf[isample]   > 0)             and
                        (0             > wf[isample+1]) and
                        (wf[isample+1] > wf[isample+2]) and
                        abs(dv-s.celldv[icell]) < dv_limit):
                        zero_xing.append(isample)
                        s.dv_values.append(dv-s.celldv[icell])
                else:
                    # look for rising edge
                    if ((wf[isample-1] < wf[isample])   and
                        (wf[isample]   < 0)             and
                        (0             < wf[isample+1]) and
                        (wf[isample+1] < wf[isample+2]) and
                        abs(dv-s.celldv[icell]) < dv_limit):
                        zero_xing.append(isample)
                        s.dv_values.append(dv-s.celldv[icell])
            # abort if incorrect number of edges is found
            if (abs(nest-len(zero_xing)) > nest/3):
                ncomplain += 1
                if ncomplain < 5:
                    rmsdv = np.array(s.dv_values).std()
                    print(f"rms dv discrepancy = {rmsdv:.1f}")
                    print("nest={} len(zero_xing)={}"
                          .format(nest, len(zero_xing)))
                    for _ in range(len(wf)):
                        print("{:d} {:.1f}".format(_, wf[_]))
                    import matplotlib.pyplot as plt
                    plt.plot(wf)
                    plt.show()
                    if ncomplain == 1:
                        import pdb ; pdb.set_trace()
                return 0
            n_correct = 0
            for i in range(len(zero_xing)-1):
                isample1 = zero_xing[i]
                isample2 = zero_xing[i+1]
                if abs((isample2-isample1)/cells_per_tcal-1.0) > 0.1:
                    # implausible distance between adjacent zero crossings
                    continue
                icell1 = (isample1 + s.tcell)%info.ncell
                icell2 = (isample2 + s.tcell)%info.ncell
                if icell1==0 or icell2==0:
                    continue
                if icell1==1023 or icell2==1023:
                    continue
                if wf[isample1]==0 or wf[isample2]==0:
                    continue
                # fit region near first zc to estimate zc time
                v = wf[isample1-5:isample1+5]
                t = tcor[isample1-5:isample1+5] - tcor[isample1]
                fitpar = np.polyfit(t, v, deg=1)
                m = fitpar[0] ; b = fitpar[1]
                tzc1 = tcor[isample1] - b/m
                # then second zc
                v = wf[isample2-5:isample2+5]
                t = tcor[isample2-5:isample2+5] - tcor[isample2]
                fitpar = np.polyfit(t, v, deg=1)
                m = fitpar[0] ; b = fitpar[1]
                tzc2 = tcor[isample2] - b/m
                tperiod = tzc2-tzc1
                # calculate correction to nominal period as a fraction
                corr_undamped = (1.0/info.fTcalFrequency)/tperiod
                if corr_undamped < 0.9 or corr_undamped > 1.1:
                    # completely implausible - skip
                    continue
                # apply damping factor
                dcorr = (corr_undamped-1.0) * damping
                limit = info.corr_limit * damping/damping0
                if dcorr > limit:
                    dcorr = limit
                elif dcorr < -limit:
                    dcorr = -limit
                corr = dcorr + 1.0
                s.corr_undamped_list.append(corr_undamped-1.0)
                # remember number of valid corrections
                n_correct += 1
                # apply from i1 to i2-1 EXclusive (was INclusive)
                assert(icell1 != icell2)
                start = isample1+1
                end = isample2
                if (start+s.tcell)%2==1:
                    start += 1
                if (end+s.tcell)%2==1:
                    end -= 1
                assert(start<end)
                # distribute correction equally into bins in the region
                for j in range(start,end):
                    jcell = (j+s.tcell)%1024
                    s.celldt[jcell] *= corr
            if (n_correct < len(zero_xing)/3):
                print("n_correct={}, len(zero_xing)={}"
                      .format(n_correct, len(zero_xing)))
                return 0
        if s.iiter==100 or (s.iiter+1)%1000==0:
            import matplotlib.pyplot as plt
            print("iiter={} rms(corr_undamped) = {:.3g} {:.3g}"
                  .format(s.iiter,
                          np.std(s.corr_undamped_list[-100:]),
                          np.std(s.corr_undamped_list)))
            cellnum = np.arange(info.ncell)
            if s.iiter==100 or (s.iiter+1)%10000==0:
                plt.plot(cellnum[0::2], 1000*s.celldt[0::2], '.')
                plt.plot(cellnum[1::2], 1000*s.celldt[1::2], '.')
                plt.grid()
                plt.xlabel(f"cellwidth [ps] : a_p iter {s.iiter}")
                plt.show()
                inl = (np.cumsum(s.celldt) -
                       s.celldt.mean()*np.arange(info.ncell))
                plt.plot(np.arange(info.ncell)[0::2], inl[0::2], '-')
                plt.grid()
                plt.xlabel("INL [ns] vs cell (even cells only)")
                plt.show()
            celldt_diff = s.celldt - s.prev_celldt
            celldt_fulldiff = s.celldt - s.initial_celldt
            s.prev_celldt = s.celldt.copy()
            plt.plot(1000*celldt_fulldiff, '.')
            plt.plot(1000*celldt_diff, '.')
            plt.grid()
            plt.xlabel("celldt [ps] difference vs analyze slope")
            plt.show()
            plt.plot(np.cumsum(1000*celldt_fulldiff), '-')
            plt.plot(np.cumsum(1000*celldt_diff), '-')
            plt.grid()
            plt.xlabel("celldt [ps] difference vs analyze slope")
            plt.show()
            print("rms celldt [ps] difference recent = "
                  f"{1000*celldt_diff.std():.5f} "
                  f"{1000*celldt_fulldiff.std():.5f} ")
        if (s.iiter+1)%100==0:
            if "celldt_historyb" not in s.__dict__:
                s.celldt_historyb = []
            s.celldt_historyb.append(s.celldt.copy())
        return 1


@dataclass
class TcalObj:
    iiter : int
    tcell : int
    idrs  : int
    ichnl : int

def do_tcal(f, chnl=0):
    # f is nominally an instance of class 'Caen'
    kh = f._kahuna

    # This should be moved back into class Caen as a method once it is
    # working properly; keeping it outside is a hack that allows
    # reload to work

    from IPython.display import Markdown as md

    tcal_all = {}  # do I need 'idrs'?  do I want 'ichnl'?
    kh.tcal_all = tcal_all
    idrs = 0
    ichnl = chnl

    info = TcalInfo(fNominalFrequency=5.0)
    s = TcalObj(
        iiter=0,
        tcell=0,
        idrs=idrs,
        ichnl=ichnl
        )
    tcal_all[(idrs,ichnl)] = s

    s.celldt = np.ones(info.ncell)/info.fNominalFrequency  # [ns]
    s.dv_list = [[] for icell in range(info.ncell)]
    s.corr_undamped_list = []

    pr = f._rdwf.pedruns
    events = pr[ichnl].events
    waves = np.array([e.drsps.copy() for e in events])
    s.cellid = np.array([e.drs_trig_cell for e in events])

    waves -= kh.wiggleshape
    gain = kh.cellgain[ichnl].copy()
    gain /= gain.mean()
    for iev in range(waves.shape[0]):
        w = waves[iev]
        w -= w.mean()
        tc = s.cellid[iev]
        g = np.roll(gain, -tc)
        w /= g
        waves[iev] = w
    s.gaincorrected = waves

    # Stefan's code iterates this nIterSlope times
    print("looping analyze_slope", flush=True)
    s.nevents = s.gaincorrected.shape[0]
    for iev in range(s.nevents):
        s.iiter = iev
        s.wf = np.array(s.gaincorrected[iev], dtype=float)
        s.wf -= np.mean(s.wf)
        s.tcell = s.cellid[iev]
        info.analyze_slope(s, calculate=(iev==s.nevents-1))
    # Stefan's code iterates this nIterPeriod times
    print("looping analyze_period", flush=True)
    s.initial_celldt = s.celldt.copy()
    s.prev_celldt = s.celldt.copy()
    iev = 0
    for iloop in range(info.nIterPeriod):
        s.iiter = iloop
        s.wf = np.array(s.gaincorrected[iev], dtype=float)
        s.wf -= np.mean(s.wf)  # added 2024-04-24
        s.tcell = s.cellid[iev]
        info.analyze_period(s, iloop)
        iev = (iev + 1)%s.cellid.shape[0]
    rmsdv = np.array(s.dv_values).std()
    print(f"rms dv discrepancy = {rmsdv:.1f}")
    # save intermediate results for debug
    historyb = np.array(s.celldt_historyb)  # consider persisting this
    
    
    
class Caen:

    # vaguely analogous to 'RocstarInit' class in 'wja_rocstar_init.py'
    nchnl = 8
    drs_num_cells = 1024

    def __init__(self):
        f = self
        f._rdwf = SimpleNamespace()
        
    def setup_tcal_acq(self, chnl=0):
        f = self
        kh = f._kahuna
        #
        kh.sine_ampl = 0.950
        dc_volts = kh.dc_volts
        if chnl in [16,17]:
            kh.sine_ampl *= 2
            dc_volts *= 2
        f.enable_sine(ampl=kh.sine_ampl, offset=dc_volts)
        f.acquire_pedruns(
            nevents=100, verbose=True, chnl=chnl,
            meanshape=kh.wiggleshape,  # chnl-by-chnl??
            use_these_peds=kh.cellpeds  # chnl-by-chnl??
        )
        pr = f._rdwf.pedruns
        kh.new_dc_volts = kh.dc_volts  # .copy()?  chnl-by-chnl?
        #
        niter = 3
        for i in range(niter):
            #
            f.iterate_sine_dc_offsets(chnl=chnl)
            #
            low = kh.low
            target_low = 200
            drs_counts_per_volt = 1900
            volts_offset = (low-target_low) / kh.drs_counts_per_volt
            kh.new_dc_volts = kh.new_dc_volts - volts_offset
            f.enable_sine(ampl=kh.sine_ampl, offset=kh.new_dc_volts)
        # on last pass, collect enough events to perform tcal
        kh.dc_volts = kh.new_dc_volts  # hmm, not in 'wja_rocstar_init'
        f.iterate_sine_dc_offsets(nevents=10000, chnl=chnl)

    def iterate_sine_dc_offsets(self, nevents=100, chnl=0):
        f = self
        kh = f._kahuna
        #
        def func(t, baseline, ampl, period, t0):
            y = baseline + ampl*np.sin(2*math.pi*(t-t0)/period)
            return y
        kh.tcal_sine_func = func
        #
        f.acquire_pedruns(
            nevents=nevents, verbose=True, chnl=chnl,
            use_these_peds=kh.cellpeds)
        pr = f._rdwf.pedruns
        #
        ncell = 1024 ; nfitpar = 4
        fitpar = np.zeros((nfitpar))
        fitrms = 0  # chnl-by-chnl?
        iev = len(pr[chnl].events)//2  # event in middle of list
        v = pr[chnl].events[iev].drsps.copy()
        v -= kh.wiggleshape
        gain = kh.cellgain.copy()
        gain /= gain.mean()
        tc = pr[chnl].events[iev].drs_trig_cell
        gain = np.roll(gain, -tc)
        v /= gain
        v = v[:200]  # use just initial part of wave for now
        t = np.arange(len(v))
        p0 = [v.mean(), (v.max()-v.min())/2, 50, 0]
        bounds = [
            (p0[0]-200, p0[0]+200),   # baseline
            (p0[1]*0.8, p0[1]*1.2),   # amplitude
            (p0[2]*0.9, p0[2]*1.1),   # period
            (-0.6*p0[2], +0.6*p0[2])  # time offset
            ]
        bounds = np.array(bounds).transpose()
        par,cov = scipy.optimize.curve_fit(
            func, t, v, p0=p0, bounds=bounds)
        print(p0)
        print(par)
        vfit = func(t, *par)
        resid = v-vfit
        rmsresid = resid.std()
        print(f"rms residual = {rmsresid}")
        #
        fitpar = par  # chnl-by-chnl?
        fitrms = rmsresid  # chnl-by-chnl?
        #
        plt.plot(v, '.-')
        plt.plot(vfit, '-')
        plt.plot(resid*10+3000, '-')
        plt.axis([0, len(v), 0, 4096])
        plt.grid()
        plt.show()
        #
        print(fitpar.round(2))
        print(f"fitrms <{fitrms.mean():.0f}>\n",
              fitrms.round(0), sep="")
        assert(fitrms.min() > 4.0)
        assert(fitrms.mean() > 4.0)
        assert(fitrms.max() < 200.0)
        baseline = fitpar[0]
        ampl = fitpar[1]
        low = baseline-ampl
        print(f"baseline <{baseline.mean():.0f}>\n",
              baseline.round(0), sep="")
        print(f"ampl <{ampl.mean():.0f}>\n", ampl.round(0), sep="")
        print(f"low <{low.mean():.0f}>\n", low.round(0), sep="")
        #
        kh.low = low
        kh.baseline = baseline
        kh.ampl = ampl
        
    def connect_awg(self):
        rm = pyvisa.ResourceManager()
        instr = rm.open_resource(
            self.awg_address, timeout=500, chunk_size=102400)
        q = instr.query("*IDN?")
        print(q)
        self.instr = instr
        qq = q.split(",")
        success = len(qq) > 1 and qq[1] == "DG4162"
        return success

    def enable_sine(self, freq=100e6, ampl=0.000, offset=0.000):
        # could do :OUTP2:STATE:OFF
        # could do :SOURCE2:FUNC DC
        if freq==0 and ampl==0:
            cmd = f"source2:apply:dc 1.0e-6,1.0e-3,{offset}"
        else:
            if freq==0: freq = 1.0e-6  # closest FG comes to DC
            if ampl==0: ampl = 1.0e-3  # closest FG comes to DC
            cmd = f"source2:apply:sin {freq},{ampl},{offset}"
        print(cmd)
        self.instr.write(cmd)
        time.sleep(0.1)
        self.instr.write(cmd)
        time.sleep(1.0)
        q = self.instr.query("source2:apply?")
        print(q)
        q = q.strip()
        if q[0] == '"':
            q = q[1:]
        if q[-1] == '"':
            q = q[:-1]
        qq = q.split(",")
        print(qq)
        assert qq[0] == "SIN" or qq[0] == "DC"
        if qq[0] == "SIN":
            assert float(qq[1]) == freq
            assert float(qq[2]) == ampl
            assert abs(float(qq[3])-offset) < 0.001
        time.sleep(0.25)
    
    def readout_n_events(self, chnl=0, n=1):
        f = self
        f.ev = None
        wddir = "/home/ashmansk/caen/wavedump-3.10.6/src"
        cmd = f"{wddir}/wavedump --wja {n}"
        print("[running] "+cmd)
        log = os.popen(cmd).read()
        print("[done]")
        self._readout_event_log = log
        ok = "\nwja batch done\n" in log
        if not ok:
            print("readout_n_events failed")
            return None
        wr = WaveReader()
        if chnl >=0 and chnl <= 15:
            fnam = f"wave_{chnl}.txt"
        elif chnl == 16:
            fnam = "TR_0_0.txt"  # 9th chnl on DRS0 : "fast trigger"
        elif chnl == 17:
            fnam = "TR_0_1.txt"  # 9th chnl on DRS1 : "fast trigger"
        else:
            raise ValueError(f"invalid channel number {chnl}")
        info, events = wr.wave_read(fnam)
        info.events = events
        return info
    
    def do_triggered_readout(self, nevents=1, draw=False):
        f = self
        n = nevents
        rdwf = self._rdwf
        f.trigev = []
        wddir = "/home/ashmansk/caen/wavedump-3.10.6/src"
        cmd = f"{wddir}/wavedump --wjatrig {n}"
        print("[running] "+cmd)
        log = os.popen(cmd).read()
        print("[done]")
        self._readout_event_log = log
        ok = "\nwja triggered batch done\n" in log
        if not ok:
            print("do_triggered_readout failed \n")
            print(log)
            return None
        wr = WaveReader()
        infos = []
        nchnl = 18  # includes 16,17 trigger channels (wja numbering)
        for chnl in range(nchnl):
            if chnl >=0 and chnl <= 15:
                fnam = f"wave_{chnl}.txt"
            elif chnl == 16:
                fnam = "TR_0_0.txt"  # 9th chnl on DRS0 : "fast trigger"
            elif chnl == 17:
                fnam = "TR_0_1.txt"  # 9th chnl on DRS1 : "fast trigger"
            else:
                raise ValueError(f"invalid channel number {chnl}")
            info, events = wr.wave_read(fnam)
            info.events = events
            infos.append(info)
        evnum = infos[0].event_number  # largest event number
        for chnl in range(len(infos)):
            assert infos[chnl].event_number == evnum
        nev = evnum + 1  # number of events
        f.trigev = nev*[None]
        for iev in range(nev):
            e = SimpleNamespace()
            f.trigev[iev] = e
            e.drsraw = np.array([
                infos[chnl].events[iev].drs
                for chnl in range(nchnl)])
            e.drs_trig_cell = np.array([
                infos[chnl].events[iev].drs_trig_cell
                for chnl in range(nchnl)])
            e.tstamp = infos[0].events[iev].tstamp
            for chnl in range(nchnl):
                assert infos[chnl].events[iev].tstamp == e.tstamp
        return f.trigev

    def correct_triggered_readout(self):
        f = self
        nchnl = f.trigev[0].drsraw.shape[0]  # should be 18
        assert nchnl == 18
        for iev in range(len(f.trigev)):
            e = f.trigev[iev]
            e.drsps = np.zeros(e.drsraw.shape)  # pedestal-subtracted
            e.drsgc = np.zeros(e.drsraw.shape)  # gain-corrected
            e.traw = np.zeros(e.drsraw.shape)   # uncorrected time axis
            e.tcor = np.zeros(e.drsraw.shape)   # corrected time axis
            e.drsu = np.zeros(e.drsraw.shape)   # drsgc resampled to tcor
            for ich in range(nchnl):
                drs = e.drsraw[ich]
                tc = f.tcal[ich]
                tcell = e.drs_trig_cell[ich]
                # subtract pedestals
                drsps = e.drsps[ich]
                pedmean = tc.cellpeds.mean()
                peds = tc.cellpeds - pedmean
                p = np.roll(peds, -tcell)
                e.drsps[ich] = drs - p
                # subtract 'wiggle shape' & apply gain correction
                e.drsgc[ich] = e.drsps[ich]
                e.drsgc[ich] -= tc.wiggleshape
                gain = tc.cellgain / tc.cellgain.mean()
                mean = e.drsgc[ich].mean()
                g = np.roll(gain, -tcell)
                w = e.drsgc[ich]
                # is 'pedmean' the right origin for gain correction?
                e.drsgc[ich] = (w - pedmean) / g + pedmean
                # create raw & corrected time axes
                cw = np.roll(tc.celldt, -(tcell-1))
                e.tcor[ich] = np.cumsum(cw) - cw[0]
                e.traw[ich] = np.arange(len(cw)) * cw.mean()
                e.drsu[ich] = np.interp(
                    e.traw[ich], e.tcor[ich], e.drsgc[ich])
        
    def load_drs_corrections(self):
        f = self
        f.drs_corrections_filename = "caen_tcal.hdf5"
        hf = h5py.File(f.drs_corrections_filename, "r")
        group_names = sorted(hf.keys())
        cal = {}
        for gn in group_names:
            m = re.match(
                r"tcal-([0-9]{8})-([0-9]{4})-ch([0-9]{2})", gn)
            assert m is not None
            mg = m.groups()
            if 0: print(gn, mg)
            yyyymmdd, hhmm, ch = mg
            cal[int(ch)] = hf[gn]
        chnmax = max(cal.keys())
        tcal = (chnmax+1)*[None]
        f.tcal = tcal
        for chnum in cal:
            tcal[chnum] = SimpleNamespace()
            tc = tcal[chnum]
            hg = cal[chnum]
            tc.idrs = hg.attrs["idrs"]
            tc.ichnl = hg.attrs["ichnl"]
            tc.tstamp = hg.attrs["tstamp"]
            tc.cellpeds = hg["cellpeds"][:]
            tc.cellgain = hg["cellgain"][:]
            tc.celldt = hg["celldt"][:]
            tc.wiggleshape = hg["wiggleshape"][:]
            tc.meanshape = hg["meanshape"][:]
        hf.close()
        
    def read_display_waveform(self, sine_enable=True, draw=True, chnl=0):
        # not sure yet what to do with 'sine_enable'
        f = self
        if sine_enable:
            ampl = 0.500
            if chnl in [16,17]: ampl *= 2
            f.enable_sine(ampl=ampl, offset=0.000)
        else:
            f.enable_sine(ampl=0, offset=0.000, freq=0)
        rdwf = self._rdwf
        info = f.readout_n_events(n=1, chnl=chnl)
        rdwf.edat = info.events[-1]
        edat = rdwf.edat
        if draw:
            plt.clf()
            plt.plot(edat.drs)
            plt.axis([None, None, 0, 4200])
            plt.grid()
            plt.show()

    def acquire_pedruns(self,
                        nevents=1000, verbose=True, chnl=0,
                        meanshape=None, use_these_peds=None):
        print("++ acquire_pedruns ++",
              time.strftime("%Y-%m-%d %H:%M:%S"))
        t0 = time.time()
        f = self
        f._rdwf.pedruns = {}
        pedruns = self._rdwf.pedruns
        pr = PedestalRun(ichannel=chnl, events=None, peds=None)
        pedruns[chnl] = pr
        print("calling readout_n_events")
        info = f.readout_n_events(chnl=chnl, n=nevents)
        print("back from readout_n_events")
        pr.events = info.events
        events = pr.events  # in old code, was list of Event objects
        #
        drs_num_cells = self.drs_num_cells
        pr.peds = np.zeros(drs_num_cells)
        pr.peds1 = np.zeros(drs_num_cells)
        if use_these_peds is not None:
            # user supplied pre-computed pedestals to apply
            pr.peds = use_these_peds.copy()  # chnl-by-chnl??
            pr.peds -= pr.peds.mean(axis=-1, keepdims=True)
        else:
            # calculate pedestals
            psum = np.zeros(drs_num_cells)
            psum1 = np.zeros(drs_num_cells)
            npsum = np.zeros(drs_num_cells)
            for e in events:
                cellid = e.drs_trig_cell
                wd = e.drs.copy()
                if meanshape is not None:
                    wd -= meanshape
                wd1 = wd.copy()
                wd -= int(round(np.mean(wd)))  # temporary?!
                l = len(wd)
                z = np.zeros(drs_num_cells-l)
                o = np.ones(l)
                onepad = np.roll(np.hstack((o,z)), cellid)
                npsum += onepad
                adcpad = np.roll(np.hstack((wd,z)), cellid)
                psum += adcpad
                # see comment re duplication in wja_rocstar_init.py
                adcpad1 = np.roll(np.hstack((wd1,z)), cellid)
                psum1 += adcpad1
            for cell in range(drs_num_cells):
                if npsum[cell]>0:
                    psum[cell] /= npsum[cell]
                    psum1[cell] /= npsum[cell]
                    pr.peds = psum  # averages to zero
                    pr.peds1 = psum1  # does not average to zero
            # done computing pedestals
        for e in events:
            e.drsps = np.zeros(e.drs.shape)
            l = len(e.drs)
            p = np.roll(pr.peds, -e.drs_trig_cell)
            drsps = e.drs - p[:l]
            e.drsps = drsps  # "drs pedestal-subtracted"
        dt = time.time() - t0
        print(f"\n-- acquire_pedruns (chnl={chnl}) --",
              time.strftime("%Y-%m-%d %H:%M:%S"),
              f"dt={dt:.1f}s\n")

    def do_pedestal_bigkahuna(self, chnl=0):
        # what exactly does the original version of this method do,
        # anyway?
        self.caen_chnl = chnl
        self.awg_address = "TCPIP0::192.168.1.21::INSTR"
        self.connect_awg()
        #
        f = self
        f._kahuna = SimpleNamespace()
        #
        f.read_display_waveform(sine_enable=True, draw=True, chnl=chnl)
        drs = f._rdwf.edat.drs
        print(f"drs mean = {np.mean(drs):.2f}")
        #
        f.read_display_waveform(sine_enable=False, draw=True, chnl=chnl)
        drs = f._rdwf.edat.drs
        print(f"drs mean = {np.mean(drs):.2f}")

    def do_dc_offset_run(self,
                         dc_value=0.500, nevents=2000,
                         verbose=True, chnl=0, meanshape=None):
        f = self
        kh = f._kahuna
        dc_volts = dc_value
        dr = DcOffsetRun(ichnl=chnl,
                         dc_volts=dc_volts,
                         psum=None)
        f.enable_sine(ampl=0, offset=dc_volts, freq=0)
        f.acquire_pedruns(
            nevents=nevents, verbose=verbose, chnl=chnl,
            meanshape=meanshape)
        dr.pr = f._rdwf.pedruns
        dr.psum = dr.pr[chnl].peds1
        print("")
        kh.dc_offset_run = dr
        kh.dc_volts = dc_volts
        #===
        psum = dr.psum
        print("min {:.0f} mean {:.0f} max {:.0f}".format(
            np.min(psum), np.mean(psum), np.max(psum)))
        drsncells = 1024
        cellpeds = dr.psum.copy()
        cellpeds -= cellpeds.mean()
        return dr

    def acquire_wiggle_shape(self, nevents=2000, chnl=0):
        f = self
        kh = f._kahuna
        dr = f.do_dc_offset_run(
            chnl=chnl, dc_value=0, nevents=nevents, verbose=False)
        # Interesting wiggle shape here: like a step every 1/6 of the
        # way across the 1024-cell readout.
        meanshape = np.zeros(self.drs_num_cells)
        v = np.array([e.drsps for e in dr.pr[chnl].events])
        vavg = v.mean(axis=0)
        meanshape[:] = vavg - vavg.mean()
        wiggle = np.zeros(6)
        wiggleshape = np.zeros(self.drs_num_cells)
        stepbins = self.drs_num_cells/len(wiggle)
        for i in range(len(wiggle)):
            lobin = math.ceil(stepbins*i)
            hibin = math.floor(stepbins*(i+1))
            wiggle[i] = meanshape[lobin:hibin].mean()
            wiggleshape[math.floor(stepbins*i):math.ceil(stepbins*(i+1))] = \
                        wiggle[i]
        kh.wiggleshape = wiggleshape
        kh.wiggle = wiggle
        kh.meanshape = meanshape
    
    def acquire_drs_cell_gains(self, nevents=2000, chnl=0):
        f = self
        kh = f._kahuna
        f.acquire_wiggle_shape(chnl=chnl, nevents=nevents)
        kh.cellgain_runs = []
        offset_voltage_values = [-0.400, -0.300, 0.300, 0.400, 0.000]
        kh.offset_voltage_values = offset_voltage_values
        kh.pedsum_mean = []
        for irun in range(len(offset_voltage_values)):
            offset = offset_voltage_values[irun]
            dr = f.do_dc_offset_run(
                chnl=chnl, dc_value=offset, nevents=nevents,
                meanshape=kh.wiggleshape, verbose=False)
            kh.cellgain_runs.append(dr)
            kh.pedsum_mean.append(dr.psum.mean())
        #
        plt.plot(kh.offset_voltage_values, kh.pedsum_mean, 'o')
        plt.grid()
        plt.xlabel(f"DC offset voltage (chnl={chnl})")
        plt.ylabel(f"DRS pedestal mean")
        m, b = np.polyfit(
            kh.offset_voltage_values,
            kh.pedsum_mean, 1)
        plt.plot(kh.offset_voltage_values,
                 m*np.array(kh.offset_voltage_values) + b,
                 '-')
        print(f"DRS counts per volt = {m:.1f}")
        kh.drs_counts_per_volt = m
        plt.show()        

    def analyze_drs_cell_gains(self, chnl=0, draw=True):
        f = self
        kh = f._kahuna
        runs = kh.cellgain_runs
        drs_expect = [595, 971, 3261, 3645, 2110]
        # check that cell-by-cell averages are in tolerance
        tolerance = 1100
        for irun in range(len(drs_expect)):
            _ = runs[irun].psum.mean()
            diff = abs(drs_expect[irun]-(_))
            if (diff > tolerance): print(f"diff = {diff}")
            _ = runs[irun].psum.min()
            diff = abs(drs_expect[irun]-(_))
            if (diff > tolerance): print(f"diff = {diff}")
            _ = runs[irun].psum.max()
            diff = abs(drs_expect[irun]-(_))
            if (diff > tolerance): print(f"diff = {diff}")
        # cell-by-cell gains using wide baseline range
        cellgain = (runs[0].psum - runs[3].psum)
        cellgain /= (drs_expect[0] - drs_expect[3])
        kh.cellgain_wide = cellgain
        # cell-by-cell gains using less wide baseline range
        cellgain = (runs[1].psum - runs[2].psum)
        cellgain /= (drs_expect[1] - drs_expect[2])
        kh.cellgain = cellgain
        try:
            # check that cell-by-cell gains are in tolerance
            g = kh.cellgain.copy()
            gw = kh.cellgain_wide.copy()
            # expected mean is x2 smaller for 9th DRS channel
            expmean = 1.0
            if chnl in [16,17]:
                expmean = 0.5
            # channel overall gain within 10%
            if abs(g.mean()-expmean) >= 0.1*expmean:
                print(f"g.mean()={g.mean()}")
            assert(abs(g.mean()-expmean) < 0.1*expmean)
            if abs(gw.mean()-expmean) >= 0.1*expmean:
                print(f"gw.mean()={gw.mean()}")
            assert(abs(gw.mean()-expmean) < 0.1*expmean)
            # no outlier cells
            assert(np.std(g) < 0.01)
            if (np.std(gw) > 0.05):
                print(f"np.std(gw) = {np.std(gw):.3f}")
            assert(np.std(gw) < 0.05)
            if (np.std(gw/g) >= 0.01):
                print(f"np.std(gw/g) = {np.std(gw/g):.4f}")
            assert(np.std(gw/g) < 0.01)
            g /= g.mean()
            gw /= gw.mean()
            assert(np.min(g) > 0.95)
            assert(np.max(g) < 1.05)
            if (np.min(gw/g) <= 0.95 or np.max(gw/g) >= 1.05):
                print(f"gw/g: min={np.min(gw/g):.3f} "
                      f"max={np.max(gw/g):.3f}")
            assert(np.min(gw/g) > 0.95)
            assert(np.max(gw/g) < 1.05)
        except AssertionError:
            print(f"chnl={chnl}")
            kh.g = g
            kh.gw = gw
            raise
        # check linearity (hi-lo)/2 vs middle DAC setting
        try:
            hi = runs[1].psum.copy()
            lo = runs[2].psum.copy()
            mi = runs[-1].psum.copy()
            av = (hi+lo)/2
            dif = av-mi
            assert(abs(dif.mean()) < 20)
            assert(abs(dif.std()) < 20)
            assert(abs(dif.min()) < 20)
            assert(abs(dif.max()) < 20)
        except AssertionError:
            print(f"chnl={chnl}")
            kh.mi = mi
            kh.av = av
            kh.div = dif
            raise
        # tentatively, the last run's results are the pedestals
        kh.cellpeds = runs[-1].psum.copy()
        # if desired, draw the 'wiggle shape'
        if draw:
            plt.plot(kh.meanshape)
            plt.plot(kh.wiggleshape, '-')
            plt.plot(kh.meanshape-kh.wiggleshape)
            plt.axis([0, self.drs_num_cells, None, None])
            plt.xlabel(f"meanshape (chnl={chnl})")
            plt.grid()
            plt.show()
        if draw:
            drsps = np.array(
                [e.drsps-kh.wiggleshape
                 for e in kh.cellgain_runs[-1].pr[chnl].events])
            iev = random.randrange(0, drsps.shape[0])  # random event
            rms = drsps[iev].std(axis=0).mean()
            print(f"chnl={chnl} iev={iev} rms={rms}")
            plt.plot(drsps[iev]-drsps[iev].mean())
            plt.xlabel("ped-subtracted waveform "
                       f"(iev={iev} chnl={chnl})")
            plt.axis([0, self.drs_num_cells, -40, 40])
            plt.grid()
            plt.show()
            plt.hist(drsps.std(axis=0), range=(0,20))
            plt.xlabel(f"histogram of rms noise (chnl={chnl})")
            plt.show()
            plt.plot(drsps.std(axis=0))
            plt.xlabel(f"rms noise vs cell (chnl={chnl})")
            plt.axis([0, self.drs_num_cells, 0, 20])
            plt.grid()
            plt.show()
            plt.plot(drsps.std(axis=0), 'o-')
            plt.xlabel(f"rms noise vs cell (chnl={chnl})")
            plt.axis([1000, 1024, 0, 20])
            plt.grid()
            plt.show()
            print("rms noise after ped sub:",
                  drsps[:,10:-10].std(axis=0).mean(axis=-1).round(2))
        
def main():
    print("wja_caen_tcal.py starting at", time.ctime())
    # This seems more complicated than needed, but I'll keep it this
    # way for now so that it roughly parallels 'main' in
    # 'wja_rocstar_init.py', where there is an elaborate 'argparse'
    ns = SimpleNamespace()  # later propagate contents to outer name space
    f = Caen()
    ns.f = f
    return ns
        
        
if __name__=="__main__":
    ns = main()
    # dump contents of 'ns' into calling namespace
    locals().update(ns.__dict__)

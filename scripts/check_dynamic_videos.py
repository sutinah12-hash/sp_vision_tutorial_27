#!/usr/bin/env python3
"""Fully decode real recordings and extract one measured encounter frame per trial."""
import argparse
import json
from pathlib import Path
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directories',nargs='+',type=Path)
    args = parser.parse_args()
    for directory in args.directories:
        video = directory/'rviz_one_click.mp4'
        probe = json.loads(subprocess.check_output([
            'ffprobe','-v','error','-show_streams','-show_format','-of','json',str(video)],text=True))
        streams = [s for s in probe['streams'] if s['codec_type']=='video']
        assert len(streams)==1
        stream = streams[0]
        assert (stream['width'],stream['height'])==(1600,1016)
        assert stream['codec_name']=='h264' and stream['r_frame_rate']=='10/1'
        decoded = subprocess.run(['ffmpeg','-hide_banner','-v','error','-i',str(video),
                                  '-f','null','-'],capture_output=True,text=True,check=True)
        assert not decoded.stderr.strip(), decoded.stderr
        obstacle = json.loads((directory/'obstacles/obstacle_metrics.json').read_text())
        gui = json.loads((directory/'gui_trigger.json').read_text())
        metrics = json.loads((directory/'data/metrics.json').read_text())
        duration = float(probe['format']['duration'])
        if metrics['action_time_s'] is not None:
            assert duration>metrics['action_time_s']+4
        frame_time = obstacle['robots']['R2']['closest_time_s']+gui['approx_click_offset_in_video_s']
        assert 0<frame_time<duration
        subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-n','-ss',str(frame_time),
                        '-i',str(video),'-frames:v','1',str(directory/'native_encounter.png')],check=True)
        report={'result':'PASS','full_decode':'no errors','codec':stream['codec_name'],
                'width':stream['width'],'height':stream['height'],'frame_rate':stream['r_frame_rate'],
                'duration_s':duration,'file_size_bytes':int(probe['format']['size']),
                'encounter_frame_time_s':frame_time,
                'frame_note':'Unedited native video frame near the measured closest R1-R2 approach; '
                             'video-to-goal offset is approximate.'}
        (directory/'video_validation.json').write_text(json.dumps(report,indent=2)+'\n')
        print(directory.name,json.dumps(report),flush=True)


if __name__=='__main__':
    main()

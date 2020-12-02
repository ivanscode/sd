from Math_Functions import *

SIM_SETTINGS = False
if SIM_SETTINGS:
	N = 5
	START_SIGMA = .01
	EXPAND_SIGMA = START_SIGMA
else:
	N = 5
	START_SIGMA = 35
	EXPAND_SIGMA = START_SIGMA


def trade_segment_points(s, s2):
	changed = True
	while changed:
		changed = False

		s2_sacrifice = s2.pop(0)
		if (meanResiduals(s + [s2_sacrifice]) - meanResiduals(s)) < (meanResiduals([s2_sacrifice] + s2) - meanResiduals(s2)):
			s.append(s2_sacrifice)
			changed = True
		else:
			s2 = [s2_sacrifice] + s2

		s_sacrifice = s.pop(-1)
		if (meanResiduals([s_sacrifice] + s2) - meanResiduals(s2)) < (meanResiduals(s + [s_sacrifice]) - meanResiduals(s)):
			s2 = [s_sacrifice] + s2
			changed = True
		else:
			s.append(s_sacrifice)


		if len(s) < N:
			return None, s + s2
		if len(s2) < N:
			return s + s2, None

	return s, s2


def LS2D(points):
	# Find segments from points
	segments = []
	pt = 0
	while pt < len(points) - N:
		print('Forming segment from points %d to %d. Start coord %s' % (pt, pt + N, str(points[pt])))
		segment = points[pt: pt + N]
		if meanResiduals(segment) > START_SIGMA:
			pt += 1
			continue
		while pt < len(points) - N:
			if meanResiduals(segment + [points[pt + N]]) > START_SIGMA:
				break
			segment.append(points[pt + N])
			pt += 1
		segments.append(segment)
		pt += N + 1

	# Trade points between the ends of segments if they correlate better
	for i in range(0, len(segments)):
		j = (i + 1) % len(segments)
		if not segments[i] or not segments[j]:
			continue
		segments[i], segments[j] = trade_segment_points(segments[i], segments[j])

	segments = list(s for s in segments if s)

	# Combine segments if their mutual residuals are below the tolerance
	changed = True
	while changed:
		changed = False
		new_segments = []
		while segments:
			segment = segments.pop(0)
			minRes = min(list(meanResiduals(s + segment) for s in segments) + [EXPAND_SIGMA])
			if minRes < EXPAND_SIGMA:
				for segment2 in segments:
					#print('Testing segments. Mean residual %f' % meanResiduals(segment + segment2))
					if fpeq(meanResiduals(segment + segment2), minRes):
						new_segments.append(segment + segment2)
						segments.remove(segment2)
						changed = True
						break
			else:
				new_segments.append(segment)
		segments = new_segments

	return segments

if __name__ == "__main__":
	a = (0, 0)
	b = (1, 1)
	c = (2, 1)
	print(meanResiduals([a, b, c]))
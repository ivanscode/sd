from Math_Functions import *

#N = 8
#START_SIGMA = .0001
#EXPAND_SIGMA = 0#START_SIGMA / 100000
#COMBINE_SIGMA = .005
#EXPAND_DISTANCE_LIMIT = .1
#TEST_COUNT = 300

N = 8
START_SIGMA = .001
EXPAND_SIGMA = .00001
COMBINE_SIGMA = 0
EXPAND_DISTANCE_LIMIT = .1
TEST_COUNT = 15

def trade_segment_points(s, s2):
	TEST_COUNT = 3000
	changed = True
	while changed:
		changed = False

		s2_sacrifice = s2.pop(0)
		if s2_sacrifice.distance_to(s2[0]) < EXPAND_DISTANCE_LIMIT and (meanResiduals(s[:-TEST_COUNT] + [s2_sacrifice]) - meanResiduals(s)) < (meanResiduals([s2_sacrifice] + s2[:TEST_COUNT]) - meanResiduals(s2)):
			s.append(s2_sacrifice)
			changed = True
		else:
			s2 = [s2_sacrifice] + s2
			continue

		s_sacrifice = s.pop(-1)
		if s_sacrifice.distance_to(s[0]) < EXPAND_DISTANCE_LIMIT and(meanResiduals([s_sacrifice] + s2[:TEST_COUNT]) - meanResiduals(s2)) < (meanResiduals(s[:-TEST_COUNT] + [s_sacrifice]) - meanResiduals(s)):
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
	print('Segmenting node measurements.')
	# Find segments from points
	segments = []
	pt = 0
	while pt < len(points) - N:
		#print('Forming segment from points %d to %d. Start coord %s' % (pt, pt + N, str(points[pt])))
		segment = points[pt: pt + N]
		if meanResiduals(segment) > START_SIGMA:
			pt += 1
			continue
		failed = False
		#for i in range(len(segment) - 1):
		#	if segment[i].distance_to(segment[i + 1]) > EXPAND_DISTANCE_LIMIT:
		#		failed = True
		#		break
		#if failed:
		#	continue

		while pt < len(points) - N:
			if segment[-1].distance_to(points[pt + N]) > EXPAND_DISTANCE_LIMIT or meanResiduals((segment + [points[pt + N]])[-TEST_COUNT:]) > EXPAND_SIGMA:
				break
			segment.append(points[pt + N])
			pt += 1
		while len(segment) > TEST_COUNT:
			if meanResiduals(segment[-TEST_COUNT-1:-2]) > meanResiduals(segment[-TEST_COUNT:]):
				break
			segment.pop(-1)
			pt -= 1

		segments.append(segment)
		pt += N + 1

	# Trade points between the ends of segments if they correlate better
	#print('Trading segment points.')
	for i in range(0, len(segments)):
		j = (i + 1) % len(segments)
		if not segments[i] or not segments[j]:
			continue
		segments[i], segments[j] = trade_segment_points(segments[i], segments[j])

	segments = list(s for s in segments if s)

	segments = combine(segments)

	return segments


SMOOTH_FACTOR = 15
THROW_OUT_WINDOW = 3
THROW_OUT_THRESHOLD = .2
#COMBINE_SIGMA = .00001
COMBINE_DISTANCE_MAX = 100

def combine(segments):
	# Combine segments if their mutual residuals are below the tolerance
	while True:
		min_sigma = 99999
		minpair = None
		for i in range(len(segments) - 1):
			for j in range(i + 1, len(segments)):
				if not any(any(p.distance_to(q) < COMBINE_DISTANCE_MAX for p in segments[i] ) for q in segments[j]):
					continue
				sigma = meanResiduals(segments[i] + segments[j])
				if sigma < min_sigma:
					min_sigma = sigma
					minpair = (i, j)
		if min_sigma < COMBINE_SIGMA:
			segments[minpair[0]] += segments.pop(minpair[1])
		else:
			break
	return segments

def segment(points):
	indices_to_pop = []
	for i in range(1, len(points) - 1):
		if points[i].distance_to(points[i-1]) > THROW_OUT_THRESHOLD and points[i].distance_to(points[i+1]) > THROW_OUT_THRESHOLD:
			indices_to_pop.insert(0, i)
	for i in indices_to_pop:
		points.pop(i)

	smoothed = []
	for i in range(0, len(points)):
		s = 0
		for j in range(-SMOOTH_FACTOR, SMOOTH_FACTOR + 1):
			index = (i - j) % len(points)
			if index < 0:
				index += len(points)
			s += points[index].r()
		smoothed.append(s)

	dividers = []
	if 		(smoothed[0] < smoothed[1] and smoothed[0] < smoothed[-1]) or \
			(smoothed[0] > smoothed[1] and smoothed[0] > smoothed[-1]) or \
			(points[0].distance_to(points[1]) > COMBINE_DISTANCE_MAX):
		dividers.append(0)

	for i in range(1, len(smoothed) - 1):
		if 		(smoothed[i] < smoothed[i + 1] and smoothed[i] < smoothed[i - 1]) or \
				(smoothed[i] > smoothed[i + 1] and smoothed[i] > smoothed[i - 1]) or \
				(points[i].distance_to(points[i + 1]) > COMBINE_DISTANCE_MAX):
			dividers.append(i)

	if 		(smoothed[-1] < smoothed[0] and smoothed[-1] < smoothed[-2]) or \
			(smoothed[-1] > smoothed[0] and smoothed[-1] > smoothed[-2]) or \
			(points[-1].distance_to(points[0]) > COMBINE_DISTANCE_MAX):
		dividers.append(len(smoothed)-1)

	segments = []
	current = []
	for i in range(0, len(smoothed)):
		current.append(points[i])
		if i in dividers:
			segments.append(current)
			current = []
	segments.append(current)
	if 0 not in dividers:
		segments[-1] += segments.pop(0)

	segments = combine(segments)
	segments = list(s for s in segments if len(s) > N)
	for i in range(0, len(segments)):
		j = (i + 1) % len(segments)
		if not segments[i] or not segments[j]:
			continue
		segments[i], segments[j] = trade_segment_points(segments[i], segments[j])

	segments = list(s for s in segments if s and len(s) > N)

	return segments

if __name__ == "__main__":
	a = (0, 0)
	b = (1, 1)
	c = (2, 1)
	print(meanResiduals([a, b, c]))
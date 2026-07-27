import type { MotionClip, UrdfRobotLike } from '../types/viewer';

export type JointLimitStatus = 'near' | 'violation';
export type JointLimitBoundary = 'lower' | 'upper';

export interface JointLimitHit {
  frameIndex: number;
  jointName: string;
  jointType: string;
  status: JointLimitStatus;
  boundary: JointLimitBoundary;
  value: number;
  lower: number;
  upper: number;
  limit: number;
  distance: number;
}

export interface JointLimitAnalysisResult {
  byFrame: Map<number, JointLimitHit[]>;
  violationFrameCount: number;
  nearFrameCount: number;
  violationHitCount: number;
  nearHitCount: number;
}

export interface JointLimitAnalysisOptions {
  nearFraction?: number;
  revoluteNearMinimum?: number;
  prismaticNearMinimum?: number;
  violationEpsilon?: number;
}

interface RuntimeJointLimit {
  jointName: string;
  jointType: string;
  jointIndex: number;
  lower: number;
  upper: number;
  nearDistance: number;
}

const DEFAULT_REVOLUTE_NEAR_MINIMUM = (2 * Math.PI) / 180;

function finiteOrInfinity(value: unknown, fallback: number): number {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : fallback;
}

function collectLimits(
  robot: UrdfRobotLike,
  clip: MotionClip,
  options: Required<JointLimitAnalysisOptions>,
): RuntimeJointLimit[] {
  const limits: RuntimeJointLimit[] = [];
  for (let jointIndex = 0; jointIndex < clip.schema.jointNames.length; jointIndex += 1) {
    const jointName = clip.schema.jointNames[jointIndex];
    const joint = (robot.joints as any)?.[jointName];
    const jointType = String(joint?.jointType ?? 'revolute').toLowerCase();
    if (
      !joint ||
      jointType === 'continuous' ||
      jointType === 'fixed' ||
      jointType === 'floating'
    ) {
      continue;
    }
    const lower = finiteOrInfinity(joint.limit?.lower, Number.NEGATIVE_INFINITY);
    const upper = finiteOrInfinity(joint.limit?.upper, Number.POSITIVE_INFINITY);
    if (!Number.isFinite(lower) && !Number.isFinite(upper)) {
      continue;
    }
    const range =
      Number.isFinite(lower) && Number.isFinite(upper) && upper > lower
        ? upper - lower
        : 0;
    const minimum =
      jointType === 'prismatic'
        ? options.prismaticNearMinimum
        : options.revoluteNearMinimum;
    limits.push({
      jointName,
      jointType,
      jointIndex,
      lower,
      upper,
      nearDistance: Math.max(minimum, range * options.nearFraction),
    });
  }
  return limits;
}

export class JointLimitAnalysisService {
  analyze(
    clip: MotionClip,
    robot: UrdfRobotLike,
    partialOptions: JointLimitAnalysisOptions = {},
  ): JointLimitAnalysisResult {
    const options: Required<JointLimitAnalysisOptions> = {
      nearFraction: partialOptions.nearFraction ?? 0.05,
      revoluteNearMinimum:
        partialOptions.revoluteNearMinimum ?? DEFAULT_REVOLUTE_NEAR_MINIMUM,
      prismaticNearMinimum: partialOptions.prismaticNearMinimum ?? 0.002,
      violationEpsilon: partialOptions.violationEpsilon ?? 1e-6,
    };
    const limits = collectLimits(robot, clip, options);
    const byFrame = new Map<number, JointLimitHit[]>();
    let violationHitCount = 0;
    let nearHitCount = 0;
    let violationFrameCount = 0;
    let nearFrameCount = 0;
    const rootCount = clip.schema.rootComponentCount;

    for (let frameIndex = 0; frameIndex < clip.frameCount; frameIndex += 1) {
      const frameBase = frameIndex * clip.stride + rootCount;
      const hits: JointLimitHit[] = [];
      for (const limit of limits) {
        const value = clip.data[frameBase + limit.jointIndex];
        if (!Number.isFinite(value)) {
          continue;
        }
        let hit: JointLimitHit | null = null;
        if (value < limit.lower - options.violationEpsilon) {
          hit = {
            frameIndex,
            jointName: limit.jointName,
            jointType: limit.jointType,
            status: 'violation',
            boundary: 'lower',
            value,
            lower: limit.lower,
            upper: limit.upper,
            limit: limit.lower,
            distance: limit.lower - value,
          };
        } else if (value > limit.upper + options.violationEpsilon) {
          hit = {
            frameIndex,
            jointName: limit.jointName,
            jointType: limit.jointType,
            status: 'violation',
            boundary: 'upper',
            value,
            lower: limit.lower,
            upper: limit.upper,
            limit: limit.upper,
            distance: value - limit.upper,
          };
        } else {
          const lowerDistance = value - limit.lower;
          const upperDistance = limit.upper - value;
          const boundary: JointLimitBoundary =
            lowerDistance <= upperDistance ? 'lower' : 'upper';
          const distance = Math.min(lowerDistance, upperDistance);
          if (distance <= limit.nearDistance) {
            hit = {
              frameIndex,
              jointName: limit.jointName,
              jointType: limit.jointType,
              status: 'near',
              boundary,
              value,
              lower: limit.lower,
              upper: limit.upper,
              limit: boundary === 'lower' ? limit.lower : limit.upper,
              distance,
            };
          }
        }
        if (hit) {
          hits.push(hit);
          if (hit.status === 'violation') {
            violationHitCount += 1;
          } else {
            nearHitCount += 1;
          }
        }
      }
      if (hits.length > 0) {
        hits.sort(
          (left, right) =>
            Number(right.status === 'violation') -
              Number(left.status === 'violation') ||
            right.distance - left.distance ||
            left.jointName.localeCompare(right.jointName),
        );
        byFrame.set(frameIndex, hits);
        if (hits.some(({ status }) => status === 'violation')) {
          violationFrameCount += 1;
        } else {
          nearFrameCount += 1;
        }
      }
    }
    return {
      byFrame,
      violationFrameCount,
      nearFrameCount,
      violationHitCount,
      nearHitCount,
    };
  }
}

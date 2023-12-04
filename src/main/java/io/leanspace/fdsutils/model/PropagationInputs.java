package io.leanspace.fdsutils.model;

import io.leanspace.flightdynamics.orekit.model.GroundStation;
import io.leanspace.flightdynamics.orekit.model.OrbitElements;
import jakarta.validation.constraints.NotNull;
import lombok.Builder;
import lombok.Data;

import java.time.Instant;
import java.util.EnumSet;
import java.util.List;

@Data
@Builder
public class PropagationInputs{

  @NotNull
  private OrbitElements.Tle tle;

  @NotNull
  private Instant startTime;

  @NotNull
  private Instant stopTime;

  private int step;

  private List<GroundStation> groundStationList;

  private EnumSet<OrbitElements.CoordinateType> coordinateTypesToUse;
}

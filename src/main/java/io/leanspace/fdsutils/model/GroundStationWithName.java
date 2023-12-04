package io.leanspace.fdsutils.model;

import io.leanspace.flightdynamics.orekit.model.GroundStation;
import jakarta.validation.constraints.NotBlank;
import jakarta.validation.constraints.NotNull;
import lombok.Builder;
import lombok.Data;

@Data
@Builder
public class GroundStationWithName {
  @NotNull
  private GroundStation groundStation;

  @NotBlank
  private String name;
}
